#include "panoptic_mapping/integration/single_tsdf_integrator.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <voxblox/integrator/merge_integration.h>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<
    TsdfIntegratorBase, SingleTsdfIntegrator, std::shared_ptr<Globals>>
    SingleTsdfIntegrator::registration_("single_tsdf");

void SingleTsdfIntegrator::Config::checkParams() const {
  checkParamConfig(projective_integrator);
  if (use_uncertainty) {
    checkParamGE(uncertainty_decay_rate, 0.f, "uncertainty_decay_rate");
    checkParamLE(uncertainty_decay_rate, 1.f, "uncertainty_decay_rate");
  }
}

void SingleTsdfIntegrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("projective_integrator", &projective_integrator);
  setupParam("use_color", &use_color);
  setupParam("use_segmentation", &use_segmentation);
  setupParam("use_uncertainty", &use_uncertainty);
  setupParam("uncertainty_decay_rate", &uncertainty_decay_rate);
  // Set this param to false since there is only one layer.
  projective_integrator.foreign_rays_clear = false;
}

SingleTsdfIntegrator::SingleTsdfIntegrator(const Config& config,
                                           std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIntegrator(config.projective_integrator, std::move(globals),
                           false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  // Setup all needed inputs.
  setRequiredInputs({InputData::InputType::kDepthImage,
                     InputData::InputType::kVertexMap,
                     InputData::InputType::kValidityImage});
  if (config_.use_color) {
    addRequiredInputs({InputData::InputType::kColorImage});
  }
  if (config_.use_segmentation) {
    addRequiredInputs({InputData::InputType::kSegmentationImage});
  }
  if (config_.use_uncertainty) {
    addRequiredInputs({InputData::InputType::kUncertaintyImage});
  }
}

void SingleTsdfIntegrator::processInput(SubmapCollection* submaps,
                                        InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK_NOTNULL(globals_->camera().get());
  CHECK(inputIsValid(*input));

  cam_config_ = &(globals_->camera()->getConfig());
  Submap* map = submaps->getSubmapPtr(submaps->getActiveFreeSpaceSubmapID());
  // Check classification layer matches the task.
  if (config_.use_uncertainty || config_.use_segmentation) {
    if (!map->hasClassLayer()) {
      LOG(WARNING) << "Can not 'use_uncertainty' or 'use_segmentation' without "
                      "a class layer, skipping frame.";
      return;
    }
    if (config_.use_uncertainty &&
        map->getClassLayer().getVoxelType() != ClassVoxelType::kUncertainty) {
      LOG(WARNING) << "Can not 'use_uncertainty' with a class layer that is "
                      "not of type 'uncertainty', skipping frame.";
      return;
    }
  }

  // Allocate all blocks in the map.
  auto t1 = std::chrono::high_resolution_clock::now();
  allocateNewBlocks(map, input);
  auto t2 = std::chrono::high_resolution_clock::now();

  // Find all active blocks that are in the field of view.
  voxblox::BlockIndexList block_lists = globals_->camera()->findVisibleBlocks(
      *map, input->T_M_C(), max_range_in_image_);
  std::vector<voxblox::BlockIndex> indices;
  indices.resize(block_lists.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = block_lists[i];
  }
  IndexGetter<voxblox::BlockIndex> index_getter(indices);
  const Transformation T_C_S = input->T_M_C().inverse() * map->getT_M_S();

  // Integrate in parallel.
  std::vector<std::future<void>> threads;
  for (int i = 0; i < config_.projective_integrator.integration_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async, [this, &index_getter, map, input, i, T_C_S]() {
          voxblox::BlockIndex index;
          while (index_getter.getNextIndex(&index)) {
            this->updateBlock(map, interpolators_[i].get(), index, T_C_S,
                              *input);
          }
        }));
  }

  // Join all threads.
  for (auto& thread : threads) {
    thread.get();
  }
  auto t3 = std::chrono::high_resolution_clock::now();

  LOG_IF(INFO, config_.verbosity >= 3)
      << "Allocate: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
      << "ms, Integrate: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
      << "ms.";
}

void SingleTsdfIntegrator::updateBlock(Submap* submap,
                                       InterpolatorBase* interpolator,
                                       const voxblox::BlockIndex& block_index,
                                       const Transformation& T_C_S,
                                       const InputData& input) const {
  // Set up preliminaries.
  if (!submap->getTsdfLayer().hasBlock(block_index)) {
    LOG_IF(WARNING, config_.verbosity >= 1)
        << "Tried to access inexistent block '" << block_index.transpose()
        << "' in submap " << submap->getID() << ".";
    return;
  }
  TsdfBlock& block = submap->getTsdfLayerPtr()->getBlockByIndex(block_index);
  bool was_updated = false;
  const float voxel_size = block.voxel_size();
  const float truncation_distance = submap->getConfig().truncation_distance;
  const int submap_id = submap->getID();
  ClassBlock::Ptr class_block;
  const bool use_class_layer =
      submap->hasClassLayer() && config_.use_segmentation;
  if (use_class_layer) {
    if (!submap->getClassLayer().hasBlock(block_index)) {
      LOG_IF(WARNING, config_.verbosity >= 1)
          << "Tried to access inexistent class block '"
          << block_index.transpose() << "' in submap " << submap->getID()
          << ".";
      return;
    }
    class_block = submap->getClassLayerPtr()->getBlockPtrByIndex(block_index);
  }

  // Update all voxels.
  for (size_t i = 0; i < block.num_voxels(); ++i) {
    TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
    ClassVoxel* class_voxel = nullptr;
    if (use_class_layer) {
      class_voxel = &class_block->getVoxelByLinearIndex(i);
    }
    const Point p_C = T_C_S * block.computeCoordinatesFromLinearIndex(
                                  i);  // Voxel center in camera frame.
    if (updateVoxel(interpolator, &voxel, p_C, input, submap_id, true,
                    truncation_distance, voxel_size, class_voxel)) {
      was_updated = true;
    }
  }

  if (was_updated) {
    block.setUpdatedAll();
  }
}

bool SingleTsdfIntegrator::updateVoxel(
    InterpolatorBase* interpolator, TsdfVoxel* voxel, const Point& p_C,
    const InputData& input, const int submap_id,
    const bool is_free_space_submap, const float truncation_distance,
    const float voxel_size, ClassVoxel* class_voxel) const {
  // Compute the signed distance. This also sets up the interpolator.
  float sdf;
  if (!computeSignedDistance(p_C, interpolator, &sdf)) {
    return false;
  }
  if (sdf < -truncation_distance) {
    return false;
  }

  // Compute the weight of the measurement.
  const float weight = computeWeight(p_C, voxel_size, truncation_distance, sdf);

  // Truncate the sdf to the truncation band.
  sdf = std::min(truncation_distance, sdf);

  // Only merge color and semantics near the surface.
  if (std::abs(sdf) < truncation_distance) {
    const Color color = interpolator->interpolateColor(input.colorImage());
    updateVoxelValues(voxel, sdf, weight, &color);

    // Update the semantic information if requested.
    if (class_voxel) {
      // Uncertainty voxels are handled differently.
      if (config_.use_uncertainty &&
          class_voxel->getVoxelType() == ClassVoxelType::kUncertainty) {
        updateUncertaintyVoxel(interpolator, input,
                               static_cast<UncertaintyVoxel*>(class_voxel));
      } else {
        updateClassVoxel(interpolator, input, class_voxel);
      }
    }
  } else {
    updateVoxelValues(voxel, sdf, weight);
  }
  return true;
}

void SingleTsdfIntegrator::updateClassVoxel(InterpolatorBase* interpolator,
                                            const InputData& input,
                                            ClassVoxel* class_voxel) const {
  // For the single TSDF case there is no belonging submap, just use the ID
  // directly.
  const int id = interpolator->interpolateID(input.idImage());
  class_voxel->incrementCount(id);
}

void SingleTsdfIntegrator::updateUncertaintyVoxel(
    InterpolatorBase* interpolator, const InputData& input,
    UncertaintyVoxel* class_voxel) const {
  // Do not update voxels which are assigned as groundtruth.
  if (class_voxel->is_ground_truth) {
    return;
  }
  // Update Uncertainty Voxel Part.
  const float uncertainty =
      interpolator->interpolateUncertainty(input.uncertaintyImage());

  // Magic uncertainty value which labels a voxel as groundtruth in the
  // uncertainty input..
  // TODO(zrene) find a better way to implement this.
  if (uncertainty == -1.0) {
    // Make sure GT voxels have zero uncertainty and entropy.
    class_voxel->counts =
        std::vector<ClassificationCount>(FixedCountVoxel::numCounts());
    // Update classification part.
    class_voxel->is_ground_truth = true;
    class_voxel->uncertainty = 0.f;
  } else {
    // Update uncertainty.
    class_voxel->uncertainty =
        config_.uncertainty_decay_rate * uncertainty +
        (1.f - config_.uncertainty_decay_rate) * class_voxel->uncertainty;
  }

  updateClassVoxel(interpolator, input, class_voxel);
}

void SingleTsdfIntegrator::allocateNewBlocks(Submap* map, InputData* input) {
  // This method also resets the depth image.
  range_image_.setZero();
  max_range_in_image_ = 0.f;

  const Transformation T_S_C = map->getT_S_M() * input->T_M_C();
  // Parse through each point to reset the depth image.
  for (int v = 0; v < input->depthImage().rows; v++) {
    for (int u = 0; u < input->depthImage().cols; u++) {
      const cv::Vec3f& vertex = input->vertexMap().at<cv::Vec3f>(v, u);
      const Point p_C(vertex[0], vertex[1], vertex[2]);
      const float ray_distance = p_C.norm();
      range_image_(v, u) = ray_distance;
      max_range_in_image_ = std::max(max_range_in_image_, ray_distance);
    }
  }
  max_range_in_image_ = std::min(max_range_in_image_, cam_config_->max_range);

  // Allocate all potential blocks.
  const float block_size = map->getTsdfLayer().block_size();
  const float block_diag_half = std::sqrt(3.f) * block_size / 2.f;
  const Transformation T_C_S = T_S_C.inverse();
  const Point camera_S = T_S_C.getPosition();  // T_S_C
  const int max_steps = std::floor((max_range_in_image_ + block_diag_half) /
                                   map->getTsdfLayer().block_size());
  for (int x = -max_steps; x <= max_steps; ++x) {
    for (int y = -max_steps; y <= max_steps; ++y) {
      for (int z = -max_steps; z <= max_steps; ++z) {
        const Point offset(x, y, z);
        const Point candidate_S = camera_S + offset * block_size;
        if (globals_->camera()->pointIsInViewFrustum(T_C_S * candidate_S,
                                                     block_diag_half)) {
          map->getTsdfLayerPtr()->allocateBlockPtrByCoordinates(candidate_S);
          if (map->hasClassLayer()) {
            map->getClassLayerPtr()->allocateBlockPtrByCoordinates(candidate_S);
          }
        }
      }
    }
  }

  // Update the bounding volume.
  map->updateBoundingVolume();
}

}  // namespace panoptic_mapping
