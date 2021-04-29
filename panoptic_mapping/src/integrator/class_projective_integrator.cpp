#include "panoptic_mapping/integrator/class_projective_integrator.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <voxblox/integrator/merge_integration.h>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<
    IntegratorBase, ClassProjectiveIntegrator, std::shared_ptr<Globals>>
    ClassProjectiveIntegrator::registration_("class_projective");

void ClassProjectiveIntegrator::Config::checkParams() const {
  checkParamConfig(pi_config);
}

void ClassProjectiveIntegrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("use_accurate_classification", &use_accurate_classification);
  setupParam("projective_integrator_config", &pi_config);
}

ClassProjectiveIntegrator::ClassProjectiveIntegrator(
    const Config& config, std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIntegrator(config.pi_config, std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void ClassProjectiveIntegrator::updateBlock(Submap* submap,
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
  const bool use_class_layer = submap->getConfig().use_class_layer;
  const bool is_free_space_submap =
      submap->getLabel() == PanopticLabel::kFreeSpace;
  voxblox::Block<TsdfVoxel>& block =
      submap->getTsdfLayerPtr()->getBlockByIndex(index);
  block.setUpdatedAll();
  // Allocate if not yet existent the class block
  ClassBlock::Ptr class_block;
  if (use_class_layer) {
    class_block = submap->getClassLayerPtr()->allocateBlockPtrByIndex(index);
  }

  const float voxel_size = block.voxel_size();
  const float truncation_distance = submap->getConfig().truncation_distance;
  const int submap_id = submap->getID();

  // Update all voxels.
  for (size_t i = 0; i < block.num_voxels(); ++i) {
    TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
    const Point p_C = T_C_S * block.computeCoordinatesFromLinearIndex(
                                  i);  // voxel center in camera frame
    // Compute distance and weight.
    int id;
    float sdf;
    float weight;
    if (!computeVoxelDistanceAndWeight(
            &sdf, &weight, &id, interpolator, p_C, color_image, id_image,
            submap_id, truncation_distance, voxel_size, is_free_space_submap)) {
      continue;
    }

    // Apply distance and weight.
    const bool point_belongs_to_this_submap = id == submap_id;
    voxel.distance =
        (voxel.distance * voxel.weight +
         std::max(std::min(truncation_distance, sdf), -truncation_distance) *
             weight) /
        (voxel.weight + weight);
    voxel.weight =
        std::min(voxel.weight + weight, config_.pi_config.max_weight);

    // Only merge color near the surface and if point belongs to the submap.
    if (std::abs(sdf) < truncation_distance) {
      voxel.color = Color::blendTwoColors(
          voxel.color, voxel.weight,
          interpolator->interpolateColor(color_image), weight);
    }

    // Update class tracking.
    if (use_class_layer) {
      ClassVoxel& class_voxel = class_block->getVoxelByLinearIndex(i);
      if (config_.use_accurate_classification) {
        // The id 0 is reserved for the belonging submap.
        if (point_belongs_to_this_submap) {
          id = 0;
        } else {
          id++;
        }
        const int counts = ++class_voxel.counts[id];
        if (counts > class_voxel.current_count) {
          class_voxel.current_count = counts;
          class_voxel.current_index = id;
        }
      } else {
        if (point_belongs_to_this_submap) {
          class_voxel.belongs_count++;
        } else {
          class_voxel.foreign_count++;
        }
      }
    }
  }
}

}  // namespace panoptic_mapping
