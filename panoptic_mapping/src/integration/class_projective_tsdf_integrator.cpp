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
  setupParam("use_binary_classification", &use_binary_classification);
  setupParam("use_instance_classification", &use_instance_classification);
  setupParam("update_only_tracked_submaps", &update_only_tracked_submaps);
  setupParam("projective_integrator", &pi_config);
}

ClassProjectiveIntegrator::ClassProjectiveIntegrator(
    const Config& config, std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIntegrator(config.pi_config, std::move(globals), false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Store class count.
  if (!config_.use_binary_classification &&
      !config_.use_instance_classification) {
    // The +1 is added because 0 is reserved for the belonging submap.
    num_classes_ = globals_->labelHandler()->numberOfLabels() + 1;
  }
}

void ClassProjectiveIntegrator::processInput(SubmapCollection* submaps,
                                             InputData* input) {
  CHECK_NOTNULL(submaps);  // Input is not used here and checked later.
  // Cache submap ids by class.
  id_to_class_.clear();
  for (const Submap& submap : *submaps) {
    id_to_class_.emplace(submap.getID(), submap.getClassID());
  }
  id_to_class_[-1] = -1;  // Used for unknown classes.
  if (config_.use_instance_classification &&
      !config_.use_binary_classification) {
    // Track the number of classes (where classes in this case are instances).
    // NOTE(schmluk): This is dangerous if submaps are de-allocated and can grow
    // arbitrarily large.
    num_classes_ = id_to_class_.size() + 1;
  }

  // Run the integration.
  ProjectiveIntegrator::processInput(submaps, input);
}

void ClassProjectiveIntegrator::updateBlock(
    Submap* submap, InterpolatorBase* interpolator,
    const voxblox::BlockIndex& block_index, const Transformation& T_C_S,
    const InputData& input) const {
  CHECK_NOTNULL(submap);
  // Set up preliminaries.
  if (!submap->getTsdfLayer().hasBlock(block_index)) {
    LOG_IF(WARNING, config_.verbosity >= 1)
        << "Tried to access inexistent block '" << block_index.transpose()
        << "' in submap " << submap->getID() << ".";
    return;
  }
  TsdfBlock& block = submap->getTsdfLayerPtr()->getBlockByIndex(block_index);
  const float voxel_size = block.voxel_size();
  const float truncation_distance = submap->getConfig().truncation_distance;
  const int submap_id = submap->getID();
  const bool is_free_space_submap =
      submap->getLabel() == PanopticLabel::kFreeSpace;
  bool was_updated = false;

  // Allocate the class block if not yet existent and get it.
  ClassBlock::Ptr class_block;
  if (submap->hasClassLayer() &&
      (!config_.update_only_tracked_submaps || submap->wasTracked())) {
    class_block =
        submap->getClassLayerPtr()->allocateBlockPtrByIndex(block_index);
  }

  // Update all voxels.
  for (size_t i = 0; i < block.num_voxels(); ++i) {
    TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
    const Point p_C = T_C_S * block.computeCoordinatesFromLinearIndex(
                                  i);  // Voxel center in camera frame.
    ClassVoxel* class_voxel = nullptr;

    if (class_block) {
      class_voxel = &class_block->getVoxelByLinearIndex(i);
    }

    if (updateVoxel(interpolator, &voxel, p_C, input, submap_id,
                    is_free_space_submap, truncation_distance, voxel_size,
                    class_voxel)) {
      was_updated = true;
    }
  }
  if (was_updated) {
    block.setUpdatedAll();
  }
}

bool ClassProjectiveIntegrator::updateVoxel(
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
  sdf = std::min(sdf, truncation_distance);

  // Only merge color and classification data near the surface.
  if (std::abs(sdf) >= truncation_distance || is_free_space_submap) {
    updateVoxelValues(voxel, sdf, weight);
  } else {
    const Color color = interpolator->interpolateColor(input.colorImage());
    updateVoxelValues(voxel, sdf, weight, &color);

    // Update the class voxel.
    if (class_voxel) {
      updateClassVoxel(interpolator, class_voxel, input, submap_id);
    }
  }
  return true;
}

void ClassProjectiveIntegrator::updateClassVoxel(InterpolatorBase* interpolator,
                                                 ClassVoxel* voxel,
                                                 const InputData& input,
                                                 const int submap_id) const {
  if (config_.use_binary_classification) {
    // Use ID 0 for belongs, 1 for does not belong.
    if (config_.use_instance_classification) {
      // Just count how often the assignments were right.
      voxel->incrementCount(
          1 - static_cast<int>(interpolator->interpolateID(input.idImage()) ==
                               submap_id));
    } else {
      // Only the class needs to match.
      auto it = id_to_class_.find(submap_id);
      auto it2 =
          id_to_class_.find(interpolator->interpolateID(input.idImage()));
      if (it != id_to_class_.end() && it2 != id_to_class_.end()) {
        voxel->incrementCount(1 - static_cast<int>(it->second == it2->second));
      } else {
        voxel->incrementCount(1);
      }
    }
  } else {
    if (config_.use_instance_classification) {
      voxel->incrementCount(interpolator->interpolateID(input.idImage()));
    } else {
      // NOTE(schmluk): id_to_class should always exist since it's created based
      // on the input.
      voxel->incrementCount(
          id_to_class_.at(interpolator->interpolateID(input.idImage())));
    }
  }
}

}  // namespace panoptic_mapping
