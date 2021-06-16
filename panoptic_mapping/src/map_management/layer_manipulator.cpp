#include "panoptic_mapping/map_management/layer_manipulator.h"

#include <algorithm>

#include <voxblox/integrator/merge_integration.h>

namespace panoptic_mapping {

void LayerManipulator::Config::checkParams() const {
  //  checkParamNE(error_threshold, 0.f, "error_threshold");
}

void LayerManipulator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("use_instance_classification", &use_instance_classification);
}

LayerManipulator::LayerManipulator(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void LayerManipulator::applyClassificationLayer(
    TsdfLayer* tsdf_layer, const ClassLayer& class_layer,
    float truncation_distance) const {
  // Check inputs.
  CHECK_NOTNULL(tsdf_layer);
  if (tsdf_layer->voxel_size() != class_layer.voxel_size() ||
      tsdf_layer->voxels_per_side() != class_layer.voxels_per_side()) {
    LOG(WARNING) << "Can not 'applyClassificationLayer' to layers that have "
                    "differing layouts.";
    return;
  }

  // Parse the tsdf layer.
  voxblox::BlockIndexList block_indices;
  tsdf_layer->getAllAllocatedBlocks(&block_indices);
  for (auto& block_index : block_indices) {
    TsdfBlock& tsdf_block = tsdf_layer->getBlockByIndex(block_index);
    const ClassBlock::ConstPtr class_block =
        class_layer.getBlockPtrByIndex(block_index);
    if (!class_block) {
      continue;
    }

    // Apply the voxel data.
    float min_distance = truncation_distance;
    bool was_updated = false;
    for (size_t i = 0; i < tsdf_block.num_voxels(); ++i) {
      TsdfVoxel& tsdf_voxel = tsdf_block.getVoxelByLinearIndex(i);
      if (tsdf_voxel.weight <= 1.0e-6) {
        continue;
      }
      if (!class_block->getVoxelByLinearIndex(i).belongsToSubmap()) {
        // TODO(schmluk): Could get proper distance by looking up the surface.
        // TODO(schmluk): Could use probability to change the weights?
        tsdf_voxel.distance = truncation_distance;
        was_updated = true;
      } else {
        min_distance = std::min(tsdf_voxel.distance, min_distance);
      }
    }
    if (min_distance > 0.f) {
      // This block does not contain useful data anymore.
      tsdf_layer->removeBlock(block_index);
    } else if (was_updated) {
      tsdf_block.setUpdatedAll();
    }
  }
}

void LayerManipulator::mergeSubmapAintoB(const Submap& A, Submap* B) const {
  // TODO(schmluk): At the moment abuses the fact that all transforms are the
  //  identity and that equal classes have equal layer layout!

  // Transform, resample, and merge the TSDF and class layers
  //  const Transformation T_B_A = B->getT_S_M() * A.getT_M_S();
  //  voxblox::mergeLayerAintoLayerB(A.getTsdfLayer(),T_B_A
  //  ,B->getTsdfLayerPtr().get());

  // TEST: just use the voxels...
  voxblox::BlockIndexList block_indices;
  A.getTsdfLayer().getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    TsdfBlock::Ptr block_B =
        B->getTsdfLayerPtr()->allocateBlockPtrByIndex(block_index);
    block_B->setUpdatedAll();
    const TsdfBlock::ConstPtr block_A =
        A.getTsdfLayer().getBlockPtrByIndex(block_index);
    for (size_t i = 0; i < block_B->num_voxels(); ++i) {
      // Merge the voxels.
      const TsdfVoxel& voxel_A = block_A->getVoxelByLinearIndex(i);
      TsdfVoxel& voxel_B = block_B->getVoxelByLinearIndex(i);
      voxblox::mergeVoxelAIntoVoxelB(voxel_A, &voxel_B);
    }
  }

  if (A.hasClassLayer() && B->hasClassLayer()) {
    voxblox::BlockIndexList block_indices;
    A.getClassLayer().getAllAllocatedBlocks(&block_indices);
    for (const auto& block_index : block_indices) {
      ClassBlock::Ptr block_B =
          B->getClassLayerPtr()->allocateBlockPtrByIndex(block_index);
      const ClassBlock::ConstPtr block_A =
          A.getClassLayer().getBlockPtrByIndex(block_index);
      for (size_t i = 0; i < block_B->num_voxels(); ++i) {
        // Merge the voxels (Just add all counts since it's integer math).
        // NOTE(schmluk): Merging makes only sense for accurate classification.
        const ClassVoxel& voxel_A = block_A->getVoxelByLinearIndex(i);
        ClassVoxel& voxel_B = block_B->getVoxelByLinearIndex(i);
        if (config_.use_instance_classification) {
          // Switch out the instance labels which are now the same.
          voxel_B.counts[0] += voxel_B.counts[A.getID() + 1];
          voxel_B.counts.erase(A.getID() + 1);
          for (const auto& id_count : voxel_A.counts) {
            if (id_count.first == B->getID() + 1) {
              voxel_B.counts[0] += id_count.second;
            } else {
              voxel_B.counts[id_count.first] += id_count.second;
            }
          }
        } else {
          for (const auto& id_count : voxel_A.counts) {
            voxel_B.counts[id_count.first] += id_count.second;
          }
          voxel_B.belongs_count += voxel_A.belongs_count;
          voxel_B.foreign_count += voxel_A.foreign_count;
        }
        voxel_B.current_count = 0;
        for (auto& id_count : voxel_B.counts) {
          if (id_count.second > voxel_B.current_count) {
            voxel_B.current_index = id_count.first;
            voxel_B.current_count = id_count.second;
          }
        }
      }
    }
  }
}

}  // namespace panoptic_mapping
