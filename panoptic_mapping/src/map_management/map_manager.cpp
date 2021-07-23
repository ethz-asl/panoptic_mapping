#include "panoptic_mapping/map_management/map_manager.h"

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace panoptic_mapping {

void MapManager::Config::checkParams() const {
  checkParamConfig(activity_manager_config);
  checkParamConfig(tsdf_registrator_config);
  checkParamConfig(layer_manipulator_config);
}

void MapManager::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);

  // Frequency.
  setupParam("prune_active_blocks_frequency", &prune_active_blocks_frequency);
  setupParam("activity_management_frequency", &activity_management_frequency);
  setupParam("change_detection_frequency", &change_detection_frequency);

  // Behavior.
  setupParam("merge_deactivated_submaps_if_possible",
             &merge_deactivated_submaps_if_possible);
  setupParam("apply_class_layer_when_deactivating_submaps",
             &apply_class_layer_when_deactivating_submaps);

  // Member configs.
  setupParam("activity_manager_config", &activity_manager_config,
             "activity_manager");
  setupParam("tsdf_registrator_config", &tsdf_registrator_config,
             "tsdf_registrator");
  setupParam("layer_manipulator_config", &layer_manipulator_config,
             "layer_manipulator");
}

MapManager::MapManager(const Config& config,
                       std::shared_ptr<SubmapCollection> map)
    : config_(config.checkValid()), map_(std::move(map)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Setup members.
  activity_manager_ =
      std::make_shared<ActivityManager>(config_.activity_manager_config);
  tsdf_registrator_ =
      std::make_shared<TsdfRegistrator>(config_.tsdf_registrator_config);
  layer_manipulator_ =
      std::make_shared<LayerManipulator>(config_.layer_manipulator_config);

  // Add all requested tasks.
  if (config_.prune_active_blocks_frequency > 0) {
    tickers_.emplace_back(config_.prune_active_blocks_frequency,
                          [this] { pruneActiveBlocks(); });
  }
  if (config_.activity_management_frequency > 0) {
    tickers_.emplace_back(config_.activity_management_frequency,
                          [this] { manageSubmapActivity(); });
  }
  if (config_.change_detection_frequency > 0) {
    tickers_.emplace_back(config_.change_detection_frequency,
                          [this] { performChangeDetection(); });
  }
}

void MapManager::tick() {
  // Increment counts for all tickers, which execute the requested actions.
  for (Ticker& ticker : tickers_) {
    ticker.tick();
  }
}

void MapManager::pruneActiveBlocks() {
  // Process all active instance and background submaps.
  auto t1 = std::chrono::high_resolution_clock::now();
  Timer timer("map_management/prune_active_blocks");
  std::stringstream info;
  std::vector<int> submaps_to_remove;
  for (Submap& submap : *map_) {
    if (submap.getLabel() == PanopticLabel::kFreeSpace || !submap.isActive()) {
      continue;
    }
    info << pruneBlocks(&submap);

    // If a submap does not contain data anymore it can be removed.
    if (submap.getTsdfLayer().getNumberOfAllocatedBlocks() == 0) {
      submaps_to_remove.emplace_back(submap.getID());
      if (config_.verbosity >= 4) {
        info << "Removed submap!";
      }
    }
  }

  // Remove submaps.
  for (int id : submaps_to_remove) {
    map_->removeSubmap(id);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  timer.Stop();
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Pruned active blocks in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
      << "ms." << info.str();
}

void MapManager::manageSubmapActivity() {
  std::unordered_set<int> active_submaps;
  if (config_.merge_deactivated_submaps_if_possible) {
    // Track de-activated submaps if requested.
    for (const Submap& submap : *map_) {
      if (submap.isActive()) {
        active_submaps.insert(submap.getID());
      }
    }
  }

  // Perform activity management.
  activity_manager_->processSubmaps(map_.get());

  // Process de-activated submaps if requested.
  if (config_.merge_deactivated_submaps_if_possible ||
      config_.apply_class_layer_when_deactivating_submaps) {
    std::unordered_set<int> deactivated_submaps;
    for (Submap& submap : *map_) {
      if (!submap.isActive() &&
          active_submaps.find(submap.getID()) != active_submaps.end()) {
        deactivated_submaps.insert(submap.getID());
      }
    }

    // Apply the class layer if requested.
    if (config_.apply_class_layer_when_deactivating_submaps) {
      for (int id : deactivated_submaps) {
        Submap* submap = map_->getSubmapPtr(id);
        submap->applyClassLayer(*layer_manipulator_);
      }
    }

    // Try to merge the submaps.
    if (config_.merge_deactivated_submaps_if_possible) {
      for (int id : deactivated_submaps) {
        int merged_id;
        int current_id = id;
        while (mergeSubmapIfPossible(current_id, &merged_id)) {
          current_id = merged_id;
        }
        if (current_id == id) {
          std::cout << "Submap " << id
                    << " was deactivated, could not be matched." << std::endl;
        }
      }
    }
  }
}

void MapManager::performChangeDetection() {
  tsdf_registrator_->checkSubmapCollectionForChange(map_.get());
}

void MapManager::finishMapping() {
  // Apply Classification
  // for (Submap& submap : *map_) {
  //   submap.applyClassLayer(*layer_manipulator_);
  // }

  // Remove all empty blocks.
  std::stringstream info;
  info << "Finished mapping: ";
  for (Submap& submap : *map_) {
    info << pruneBlocks(&submap);
  }
  LOG(INFO) << info.str();

  // Deactivate last submaps.
  // for (Submap& submap : *map_) {
  //   if (submap.isActive()) {
  //     submap.finishActivePeriod();
  //     mergeSubmapIfPossible(submap.getID());
  //   }
  // }

  // Merge what is possible.
  // bool merged_something = true;
  // while (merged_something) {
  //   for (Submap& submap : *map_) {
  //     merged_something = mergeSubmapIfPossible(submap.getID());
  //     if (merged_something) {
  //       continue;
  //     }
  //   }
  // }

  // Finish submaps.
  //  for (Submap& submap : *map_) {

  //  }
}

bool MapManager::mergeSubmapIfPossible(int submap_id, int* merged_id) {
  // Use on inactive submaps, checks for possible matches with other inactive
  // submaps.
  if (!map_->submapIdExists(submap_id)) {
    return false;
  }

  // Setup.
  Submap* submap = map_->getSubmapPtr(submap_id);
  if (submap->isActive()) {
    // Active submaps need first to be de-activated.
    submap->finishActivePeriod();
  }
  const float acceptance_count =
      std::min(static_cast<float>(
                   config_.tsdf_registrator_config.match_acceptance_points),
               config_.tsdf_registrator_config.match_acceptance_percentage *
                   submap->getIsoSurfacePoints().size());

  // Find all potential matches.
  for (Submap& other : *map_) {
    if (other.isActive() || other.getClassID() != submap->getClassID() ||
        other.getID() == submap->getID() ||
        !submap->getBoundingVolume().intersects(other.getBoundingVolume())) {
      continue;
    }

    float matching_points;
    if (!tsdf_registrator_->submapsConflict(*submap, other, &matching_points)) {
      if (matching_points > acceptance_count) {
        // It's a match, merge the submap into the candidate.
        layer_manipulator_->mergeSubmapAintoB(*submap, &other);
        std::cout << "Merged Submap " << submap->getID() << " into "
                  << other.getID() << "." << std::endl;
        map_->removeSubmap(submap_id);
        if (merged_id) {
          *merged_id = other.getID();
        }
        return true;
      }
    }
  }
  return false;
}

std::string MapManager::pruneBlocks(Submap* submap) const {
  auto t1 = std::chrono::high_resolution_clock::now();
  // Setup.
  ClassLayer* class_layer = nullptr;
  if (submap->hasClassLayer()) {
    class_layer = submap->getClassLayerPtr().get();
  }
  TsdfLayer* tsdf_layer = submap->getTsdfLayerPtr().get();
  MeshLayer* mesh_layer = submap->getMeshLayerPtr().get();
  const int voxel_indices = std::pow(submap->getConfig().voxels_per_side, 3);
  int count = 0;

  // Remove all blocks that don't have any belonging voxels.
  voxblox::BlockIndexList block_indices;
  tsdf_layer->getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    const ClassBlock* class_block = nullptr;
    if (class_layer) {
      if (class_layer->hasBlock(block_index)) {
        class_block = &class_layer->getBlockByIndex(block_index);
      }
    }
    const TsdfBlock& tsdf_block = tsdf_layer->getBlockByIndex(block_index);
    bool has_beloning_voxels = false;

    // Check all voxels.
    for (int voxel_index = 0; voxel_index < voxel_indices; ++voxel_index) {
      if (tsdf_block.getVoxelByLinearIndex(voxel_index).weight >= 1e-6) {
        if (class_block) {
          if (classVoxelBelongsToSubmap(
                  class_block->getVoxelByLinearIndex(voxel_index))) {
            has_beloning_voxels = true;
            break;
          }
        } else {
          has_beloning_voxels = true;
          break;
        }
      }
    }

    // Prune blocks.
    if (!has_beloning_voxels) {
      if (class_layer) {
        class_layer->removeBlock(block_index);
      }
      tsdf_layer->removeBlock(block_index);
      mesh_layer->removeMesh(block_index);
      count++;
    }
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::stringstream ss;
  if (count > 0 && config_.verbosity >= 4) {
    ss << "\nPruned " << count << " blocks from submap " << submap->getID()
       << " (" << submap->getName() << ") in "
       << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
       << "ms.";
  }
  return ss.str();
}

void MapManager::Ticker::tick() {
  // Perform 'action' every 'max_ticks' ticks.
  current_tick_++;
  if (current_tick_ >= max_ticks_) {
    action_();
    current_tick_ = 0;
  }
}

}  // namespace panoptic_mapping
