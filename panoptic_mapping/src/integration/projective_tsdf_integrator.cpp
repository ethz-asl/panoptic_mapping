#include "panoptic_mapping/integration/projective_tsdf_integrator.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <voxblox/integrator/merge_integration.h>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<
    TsdfIntegratorBase, ProjectiveIntegrator, std::shared_ptr<Globals>>
    ProjectiveIntegrator::registration_("projective");

void ProjectiveIntegrator::Config::checkParams() const {
  checkParamGT(integration_threads, 0, "integration_threads");
  checkParamGT(max_weight, 0.f, "max_weight");
}

void ProjectiveIntegrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("use_weight_dropoff", &use_weight_dropoff);
  setupParam("use_constant_weight", &use_constant_weight);
  setupParam("foreign_rays_clear", &foreign_rays_clear);
  setupParam("max_weight", &max_weight);
  setupParam("interpolation_method", &interpolation_method);
  setupParam("integration_threads", &integration_threads);
}

ProjectiveIntegrator::ProjectiveIntegrator(const Config& config,
                                           std::shared_ptr<Globals> globals)
    : config_(config.checkValid()), TsdfIntegratorBase(std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  // Request all inputs.
  setRequiredInputs({InputData::InputType::kColorImage,
                     InputData::InputType::kDepthImage,
                     InputData::InputType::kSegmentationImage,
                     InputData::InputType::kVertexMap});

  // Setup the interpolators (one for each thread).
  for (int i = 0; i < config_.integration_threads; ++i) {
    interpolators_.emplace_back(
        config_utilities::Factory::create<InterpolatorBase>(
            config_.interpolation_method));
  }

  // Allocate range image.
  range_image_ = Eigen::MatrixXf(globals_->camera()->getConfig().height,
                                 globals_->camera()->getConfig().width);
}

void ProjectiveIntegrator::processInput(SubmapCollection* submaps,
                                        InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK_NOTNULL(globals_->camera().get());
  CHECK(inputIsValid(*input));

  // Allocate all blocks in corresponding submaps.
  Timer alloc_timer("tsdf_integration/allocate_blocks");
  cam_config_ = &(globals_->camera()->getConfig());
  allocateNewBlocks(submaps, input);
  alloc_timer.Stop();

  // Find all active blocks that are in the field of view.
  // Note(schmluk): This could potentially also be included in the parallel part
  // but is already almost instantaneous.
  Timer find_timer("tsdf_integration/find_blocks");
  std::unordered_map<int, voxblox::BlockIndexList> block_lists =
      globals_->camera()->findVisibleBlocks(*submaps, input->T_M_C(), true);
  std::vector<int> id_list;
  id_list.reserve(block_lists.size());
  for (const auto& id_blocklist_pair : block_lists) {
    id_list.emplace_back(id_blocklist_pair.first);
  }
  find_timer.Stop();

  // Integrate in parallel.
  Timer int_timer("tsdf_integration/integration");
  SubmapIndexGetter index_getter(id_list);
  std::vector<std::future<void>> threads;
  for (int i = 0; i < config_.integration_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async,
        [this, &index_getter, &block_lists, submaps, input, i]() {
          int index;
          while (index_getter.getNextIndex(&index)) {
            this->updateSubmap(submaps->getSubmapPtr(index),
                               interpolators_[i].get(), block_lists.at(index),
                               input->T_M_C(), input->colorImage(),
                               *(input->idImage()));
          }
        }));
  }

  // Join all threads.
  for (auto& thread : threads) {
    thread.get();
  }
  int_timer.Stop();
}

void ProjectiveIntegrator::updateSubmap(
    Submap* submap, InterpolatorBase* interpolator,
    const voxblox::BlockIndexList& block_indices, const Transformation& T_M_C,
    const cv::Mat& color_image, const cv::Mat& id_image) const {
  CHECK_NOTNULL(submap);
  // Update.
  Transformation T_C_S =
      T_M_C.inverse() * submap->getT_M_S();  // p_C = T_C_M * T_M_S * p_S
  for (const auto& index : block_indices) {
    updateBlock(submap, interpolator, index, T_C_S, color_image, id_image);
  }
}

void ProjectiveIntegrator::updateBlock(Submap* submap,
                                       InterpolatorBase* interpolator,
                                       const voxblox::BlockIndex& index,
                                       const Transformation& T_C_S,
                                       const cv::Mat& color_image,
                                       const cv::Mat& id_image) const {
  CHECK_NOTNULL(submap);
  // set up preliminaries
  if (!submap->getTsdfLayer().hasBlock(index)) {
    LOG(WARNING) << "Tried to access inexistent block '" << index.transpose()
                 << "' in submap " << submap->getID() << ".";
    return;
  }
  voxblox::Block<TsdfVoxel>& block =
      submap->getTsdfLayerPtr()->getBlockByIndex(index);
  block.setUpdatedAll();
  const float voxel_size = block.voxel_size();
  const float truncation_distance = submap->getConfig().truncation_distance;
  const int submap_id = submap->getID();

  // update all voxels
  for (size_t i = 0; i < block.num_voxels(); ++i) {
    voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
    const Point p_C = T_C_S * block.computeCoordinatesFromLinearIndex(
                                  i);  // voxel center in camera frame
    // Compute distance and weight.
    int id;
    float sdf;
    float weight;
    const bool is_free_space_submap =
        submap->getLabel() == PanopticLabel::kFreeSpace;
    if (!computeVoxelDistanceAndWeight(
            &sdf, &weight, &id, interpolator, p_C, color_image, id_image,
            submap_id, truncation_distance, voxel_size, is_free_space_submap)) {
      continue;
    }

    // Apply distance, color, and weight.
    const bool point_belongs_to_this_submap = id == submap_id;
    if (point_belongs_to_this_submap || is_free_space_submap) {
      voxel.distance = (voxel.distance * voxel.weight +
                        std::max(std::min(truncation_distance, sdf),
                                 -1.f * truncation_distance) *
                            weight) /
                       (voxel.weight + weight);
      voxel.weight = std::min(voxel.weight + weight, config_.max_weight);
      // only merge color near the surface and if point belongs to the submap.
      if (std::abs(sdf) < truncation_distance && point_belongs_to_this_submap) {
        voxel.color = Color::blendTwoColors(
            voxel.color, voxel.weight,
            interpolator->interpolateColor(color_image), weight);
      }
    } else {
      // Voxels that don't belong to the submap are 'cleared' if they are in
      // front of the surface.
      if (sdf > 0) {
        voxel.distance =
            (voxel.distance * voxel.weight + truncation_distance * weight) /
            (voxel.weight + weight);
        voxel.weight = std::min(voxel.weight + weight, config_.max_weight);
      }
    }
  }
}

bool ProjectiveIntegrator::computeVoxelDistanceAndWeight(
    float* sdf, float* weight, int* id, InterpolatorBase* interpolator,
    const Point& p_C, const cv::Mat& color_image, const cv::Mat& id_image,
    int submap_id, float truncation_distance, float voxel_size,
    bool is_free_space_submap) const {
  // Skip voxels that are too far or too close.
  if (p_C.z() < 0.0) {
    return false;
  }
  const float distance_to_voxel = p_C.norm();
  if (distance_to_voxel < cam_config_->min_range ||
      distance_to_voxel > cam_config_->max_range) {
    return false;
  }

  // Project the current voxel into the range image, only count points that fall
  // fully into the image.
  float u;
  float v;
  if (!globals_->camera()->projectPointToImagePlane(p_C, &u, &v)) {
    return false;
  }

  // Set up the interpolator and check whether this is a clearing or an updating
  // measurement.
  *id = interpolator->computeWeights(u, v, range_image_, id_image);
  const bool point_belongs_to_this_submap = *id == submap_id;
  if (!point_belongs_to_this_submap &&
      !(config_.foreign_rays_clear || is_free_space_submap)) {
    return false;
  }

  //  Compute the signed distance.
  const float distance_to_surface =
      interpolator->interpolateDepth(range_image_);
  if (distance_to_surface > cam_config_->max_range ||
      distance_to_surface < cam_config_->min_range) {
    return false;
  }
  const float new_sdf = distance_to_surface - distance_to_voxel;
  if (new_sdf < -truncation_distance) {
    return false;
  }

  // Compute the weight of the measurement.
  // This approximates the number of rays that would hit this voxel.
  float observation_weight =
      cam_config_->fx * cam_config_->fy * std::pow(voxel_size / p_C.z(), 2.f);

  // Weight reduction with distance squared (according to sensor noise models).
  if (!config_.use_constant_weight) {
    observation_weight /= std::pow(p_C.z(), 2.f);
  }

  // Apply weight drop-off if appropriate.
  if (config_.use_weight_dropoff && point_belongs_to_this_submap) {
    const voxblox::FloatingPoint dropoff_epsilon = voxel_size;
    if (new_sdf < -dropoff_epsilon) {
      observation_weight = observation_weight *
                           (truncation_distance + new_sdf) /
                           (truncation_distance - dropoff_epsilon);
      observation_weight = std::max(observation_weight, 0.f);
    }
  }

  // Results (id is already updated).
  *sdf = new_sdf;
  *weight = observation_weight;
  return true;
}

void ProjectiveIntegrator::allocateNewBlocks(SubmapCollection* submaps,
                                             InputData* input) {
  // This method also resets the depth image.
  range_image_.setZero();
  max_range_in_image_ = 0.f;

  // Parse through each point to allocate instance + background blocks.
  std::unordered_set<Submap*> touched_submaps;
  for (int v = 0; v < input->depthImage().rows; v++) {
    for (int u = 0; u < input->depthImage().cols; u++) {
      const cv::Vec3f& vertex = input->vertexMap().at<cv::Vec3f>(v, u);
      const Point p_C(vertex[0], vertex[1], vertex[2]);
      const float ray_distance = p_C.norm();
      range_image_(v, u) = ray_distance;
      if (ray_distance > cam_config_->max_range ||
          ray_distance < cam_config_->min_range) {
        continue;
      }
      max_range_in_image_ = std::max(max_range_in_image_, ray_distance);
      const int id = input->idImage()->at<int>(v, u);
      if (submaps->submapIdExists(id)) {
        Submap* submap = submaps->getSubmapPtr(id);
        const Point p_S = submap->getT_S_M() * input->T_M_C() *
                          p_C;  // p_S = T_S_M * T_M_C * p_C
        const voxblox::BlockIndex index =
            submap->getTsdfLayer().computeBlockIndexFromCoordinates(p_S);
        submap->getTsdfLayerPtr()->allocateBlockPtrByIndex(index);
        if (submap->hasClassLayer()) {
          // NOTE(schmluk): The projective integrator does not use the class
          // layer but was added here for simplicity.
          submap->getClassLayerPtr()->allocateBlockPtrByIndex(index);
        }
        touched_submaps.insert(submap);
      }
    }
  }

  // Allocate all potential free space blocks.
  if (submaps->submapIdExists(submaps->getActiveFreeSpaceSubmapID())) {
    Submap* space =
        submaps->getSubmapPtr(submaps->getActiveFreeSpaceSubmapID());
    const float block_size = space->getTsdfLayer().block_size();
    const float block_diag_half = std::sqrt(3.f) * block_size / 2.f;
    const Transformation T_C_S = input->T_M_C().inverse() * space->getT_M_S();
    const Point camera_S = T_C_S.inverse().getPosition();  // T_S_C
    const int max_steps = std::floor((max_range_in_image_ + block_diag_half) /
                                     space->getTsdfLayer().block_size());
    for (int x = -max_steps; x <= max_steps; ++x) {
      for (int y = -max_steps; y <= max_steps; ++y) {
        for (int z = -max_steps; z <= max_steps; ++z) {
          const Point offset(x, y, z);
          const Point candidate_S = camera_S + offset * block_size;
          if (globals_->camera()->pointIsInViewFrustum(T_C_S * candidate_S,
                                                       block_diag_half)) {
            space->getTsdfLayerPtr()->allocateBlockPtrByCoordinates(
                candidate_S);
          }
        }
      }
    }
    space->getBoundingVolumePtr()->update();
  }

  // Update all bounding volumes. This is currently done in every integration
  // step since it's not too expensive and won't do anything if no new block
  // was allocated.
  for (auto& submap : touched_submaps) {
    submap->updateBoundingVolume();
  }
}

}  // namespace panoptic_mapping
