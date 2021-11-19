#include "panoptic_mapping/map_management/layer_manipulator.h"

#include <algorithm>

#include <voxblox/integrator/esdf_integrator.h>
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
        class_layer.getBlockConstPtrByIndex(block_index);
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
    if (min_distance == truncation_distance) {
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

  if (!(A.hasClassLayer() && B->hasClassLayer())) {
    LOG(WARNING) << "Currently can only fuse submaps with class layers.";
    return;
  }

  // Currently just use the voxels...
  voxblox::BlockIndexList block_indices;
  A.getTsdfLayer().getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    TsdfBlock::Ptr block_B =
        B->getTsdfLayerPtr()->allocateBlockPtrByIndex(block_index);
    block_B->setUpdatedAll();
    const TsdfBlock::ConstPtr block_A =
        A.getTsdfLayer().getBlockPtrByIndex(block_index);
    ClassBlock::ConstPtr class_A =
        A.getClassLayer().getBlockConstPtrByIndex(block_index);
    ClassBlock::Ptr class_B =
        B->getClassLayerPtr()->allocateBlockPtrByIndex(block_index);

    for (size_t i = 0; i < block_B->num_voxels(); ++i) {
      // Merge the voxels.
      const TsdfVoxel& voxel_A = block_A->getVoxelByLinearIndex(i);
      TsdfVoxel& voxel_B = block_B->getVoxelByLinearIndex(i);
      const ClassVoxel* class_voxel_A = nullptr;
      ClassVoxel* class_voxel_B = nullptr;
      bool belongs_A = true;
      if (class_A) {
        class_voxel_A = &class_A->getVoxelByLinearIndex(i);
        belongs_A = class_voxel_A->belongsToSubmap();
      }
      bool belongs_B = true;
      if (class_B) {
        class_voxel_B = &class_B->getVoxelByLinearIndex(i);
        belongs_B = class_voxel_B->belongsToSubmap();
      }
      if (belongs_A && belongs_B) {
        voxblox::mergeVoxelAIntoVoxelB(voxel_A, &voxel_B);
        if (class_voxel_A && class_voxel_B) {
          // NOTE(schmluk): Currently just implemented for the binary case with
          // normalization!
          // TODO
          // const int belongs_count =
          //     static_cast<int>(class_voxel_A->belongs_count) +
          //     static_cast<int>(class_voxel_B->belongs_count);
          // const int foreign_count =
          //     static_cast<int>(class_voxel_A->foreign_count) +
          //     static_cast<int>(class_voxel_B->foreign_count);
          // const int normalization =
          //     (belongs_count > 255 || foreign_count > 255) ? 2 : 1;
          // class_voxel_B->belongs_count =
          //     static_cast<ClassVoxel::Counter>(belongs_count /
          //     normalization);
          // class_voxel_B->foreign_count =
          //     static_cast<ClassVoxel::Counter>(foreign_count /
          //     normalization);
        }
      } else if (belongs_A) {
        // voxel_B.distance = voxel_A.distance;
        // voxel_B.weight = voxel_A.weight;
        // if (class_voxel_A && class_voxel_B) {
        //   class_voxel_B->belongs_count = class_voxel_A->belongs_count;
        //   class_voxel_B->foreign_count = class_voxel_A->foreign_count;
        // }
        // voxel_B.color = voxel_A.color;
      }
    }
  }
}

void LayerManipulator::unprojectTsdfLayer(TsdfLayer* tsdf_layer) const {
  CHECK_NOTNULL(tsdf_layer);
  voxblox::EsdfIntegrator::Config config;
  voxblox::Layer<voxblox::EsdfVoxel> esdf_layer(tsdf_layer->voxel_size(),
                                                tsdf_layer->voxels_per_side());
  voxblox::EsdfIntegrator integrator(config, tsdf_layer, &esdf_layer);
  integrator.updateFromTsdfLayerBatch();
  voxblox::BlockIndexList block_indices;
  tsdf_layer->getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    TsdfBlock& tsdf_block = tsdf_layer->getBlockByIndex(block_index);
    const voxblox::Block<voxblox::EsdfVoxel>& esdf_block =
        esdf_layer.getBlockByIndex(block_index);
    for (size_t i = 0; i < tsdf_block.num_voxels(); ++i) {
      tsdf_block.getVoxelByLinearIndex(i).distance =
          esdf_block.getVoxelByLinearIndex(i).distance;
    }
  }
}

}  // namespace panoptic_mapping
