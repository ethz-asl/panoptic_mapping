#include "panoptic_mapping/integrator/single_tsdf_integrator.h"

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

config_utilities::Factory::RegistrationRos<IntegratorBase, SingleTsdfIntegrator,
                                           std::shared_ptr<Globals>>
    SingleTsdfIntegrator::registration_("single_tsdf");

void SingleTsdfIntegrator::Config::checkParams() const {
  checkParamConfig(pi_config);
}

void SingleTsdfIntegrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("projective_integrator_config", &pi_config);
}

SingleTsdfIntegrator::SingleTsdfIntegrator(const Config& config,
                                           std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIntegrator(config.pi_config, std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void SingleTsdfIntegrator::processInput(SubmapCollection* submaps,
                                        InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK_NOTNULL(globals_->camera().get());
  CHECK(inputIsValid(*input));

  // Allocate all blocks in corresponding submaps.
  cam_config_ = &(globals_->camera()->getConfig());
  Submap* map = submaps->getSubmapPtr(submaps->getActiveFreeSpaceSubmapID());
  auto t1 = std::chrono::high_resolution_clock::now();
  allocateNewBlocks(map, input);
  auto t2 = std::chrono::high_resolution_clock::now();

  // Find all active blocks that are in the field of view.
  voxblox::BlockIndexList block_lists =
      globals_->camera()->findVisibleBlocks(*map, input->T_M_C());
  std::vector<voxblox::BlockIndex> indices;
  indices.resize(block_lists.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = block_lists[i];
  }
  IndexGetter<voxblox::BlockIndex> index_getter(indices);
  Transformation T_C_S =
      input->T_M_C().inverse() * map->getT_M_S();  // p_C = T_C_M * T_M_S * p_S

  // Integrate in parallel.
  std::vector<std::future<bool>> threads;
  for (int i = 0; i < config_.pi_config.integration_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async, [this, &index_getter, map, input, i, T_C_S]() {
          voxblox::BlockIndex index;
          while (index_getter.getNextIndex(&index)) {
            this->updateBlock(map, interpolators_[i].get(), index, T_C_S,
                              input->colorImage(), *(input->idImage()));
          }
          return true;
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
      voxel.weight =
          std::min(voxel.weight + weight, config_.pi_config.max_weight);
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
        voxel.weight =
            std::min(voxel.weight + weight, config_.pi_config.max_weight);
      }
    }
  }
}

void SingleTsdfIntegrator::allocateNewBlocks(Submap* map, InputData* input) {
  // This method also resets the depth image.
  range_image_.setZero();
  max_range_in_image_ = 0.f;

  const Transformation T_S_C = map->getT_S_M() * input->T_M_C();

  // Parse through each point to allocate instance + background blocks.
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
    }
  }

  // Allocate all potential space blocks.
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
        }
      }
    }
  }

  // Update all bounding volumes. This is currently done in every integration
  // step since it's not too expensive and won't do anything if no new block
  // was allocated.
  map->updateBoundingVolume();
}

}  // namespace panoptic_mapping
