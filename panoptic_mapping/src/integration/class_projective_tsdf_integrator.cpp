#include "panoptic_mapping/integration/class_projective_tsdf_integrator.h"

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
    TsdfIntegratorBase, ClassProjectiveIntegrator, std::shared_ptr<Globals>>
    ClassProjectiveIntegrator::registration_("class_projective");

void ClassProjectiveIntegrator::Config::checkParams() const {
  checkParamConfig(pi_config);
}

void ClassProjectiveIntegrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("use_accurate_classification", &use_accurate_classification);
  setupParam("use_instance_classification", &use_instance_classification);
  setupParam("projective_integrator_config", &pi_config);
}

ClassProjectiveIntegrator::ClassProjectiveIntegrator(
    const Config& config, std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIntegrator(config.pi_config, std::move(globals), false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void ClassProjectiveIntegrator::processInput(SubmapCollection* submaps,
                                             InputData* input) {
  CHECK_NOTNULL(submaps);  // Input is not used here and checked later.
  // Cache submap ids by class.
  id_to_class_.clear();
  for (const Submap& submap : *submaps) {
    id_to_class_.emplace(submap.getID(), submap.getClassID());
  }

  // Run the integration.
  ProjectiveIntegrator::processInput(submaps, input);
}

void ClassProjectiveIntegrator::updateBlock(
    Submap* submap, InterpolatorBase* interpolator,
    const voxblox::BlockIndex& block_index, const Transformation& T_C_S,
    const InputData& input) const {
  CHECK_NOTNULL(submap);
  // set up preliminaries
  if (!submap->getTsdfLayer().hasBlock(block_index)) {
    LOG(WARNING) << "Tried to access inexistent block '"
                 << block_index.transpose() << "' in submap " << submap->getID()
                 << ".";
    return;
  }
  const bool use_class_layer = submap->hasClassLayer();
  const bool is_free_space_submap =
      submap->getLabel() == PanopticLabel::kFreeSpace;
  voxblox::Block<TsdfVoxel>& block =
      submap->getTsdfLayerPtr()->getBlockByIndex(block_index);
  block.setUpdatedAll();
  // Allocate if not yet existent the class block
  ClassBlock::Ptr class_block;
  if (use_class_layer) {
    class_block =
        submap->getClassLayerPtr()->allocateBlockPtrByIndex(block_index);
  }

  const float voxel_size = block.voxel_size();
  const float truncation_distance = submap->getConfig().truncation_distance;
  const int submap_id = submap->getID();

  // Update all voxels.
  for (size_t i = 0; i < block.num_voxels(); ++i) {
    TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
    const Point p_C = T_C_S * block.computeCoordinatesFromLinearIndex(
                                  i);  // voxel center in camera frame

    // Compute the signed distance. This also sets up the interpolator.
    float sdf;
    if (!computeSignedDistance(p_C, interpolator, &sdf)) {
      continue;
    }
    if (sdf < -truncation_distance) {
      continue;
    }

    // Check whether this is a clearing or an updating measurement.
    int id = interpolator->interpolateID(input.idImage());

    // Compute the weight of the measurement.
    const float weight =
        computeWeight(p_C, voxel_size, truncation_distance, sdf);

    // Truncate the sdf to the truncation band.
    sdf = std::min(truncation_distance, sdf);

    // Only merge color and class information near the surface and if point
    // belongs to the submap.
    if (std::abs(sdf) > truncation_distance) {
      updateVoxelValues(&voxel, sdf, weight);
    } else {
      const Color color = interpolator->interpolateColor(input.colorImage());
      updateVoxelValues(&voxel, sdf, weight, &color);
      // Update class tracking.
      if (use_class_layer) {
        ClassVoxel& class_voxel = class_block->getVoxelByLinearIndex(i);
        // The id 0 is reserved for the belonging submap, others are offset
        // by 1.
        if (config_.use_instance_classification) {
          if (id == submap_id) {
            id = 0;
          } else {
            id++;
          }
        } else {
          // Use class labels.
          auto it = id_to_class_.find(id);
          if (it == id_to_class_.end()) {
            // Unknown labels will be ignored.
            continue;
          }
          id = it->second;
          if (id == submap->getClassID()) {
            id = 0;
          }
        }
        if (config_.use_accurate_classification) {
          const ClassVoxel::Counter counts = ++(class_voxel.counts[id]);
          if (counts > class_voxel.belongs_count) {
            class_voxel.belongs_count = counts;
            class_voxel.current_index = id;
          }
        } else {
          if (id == 0) {
            class_voxel.belongs_count++;
          } else {
            class_voxel.foreign_count++;
          }
        }
      }
    }
  }
}

}  // namespace panoptic_mapping
