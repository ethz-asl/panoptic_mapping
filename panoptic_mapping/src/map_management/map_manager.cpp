#include "panoptic_mapping/map_management/map_manager.h"

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<MapManagerBase, MapManager>
    MapManager::registration_("submaps");

void MapManager::Config::checkParams() const {
  checkParamConfig(activity_manager_config);
  checkParamConfig(tsdf_registrator_config);
  checkParamConfig(layer_manipulator_config);
}

void MapManager::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("prune_active_blocks_frequency", &prune_active_blocks_frequency);
  setupParam("activity_management_frequency", &activity_management_frequency);
  setupParam("prune_tracked_instances_frequency",
             &prune_tracked_instances_frequency);
  setupParam("change_detection_frequency", &change_detection_frequency);
  setupParam("merge_deactivated_submaps_if_possible",
             &merge_deactivated_submaps_if_possible);
  setupParam("apply_class_layer_when_deactivating_submaps",
             &apply_class_layer_when_deactivating_submaps);
  setupParam("activity_manager_config", &activity_manager_config,
             "activity_manager");
  setupParam("tsdf_registrator_config", &tsdf_registrator_config,
             "tsdf_registrator");
  setupParam("layer_manipulator_config", &layer_manipulator_config,
             "layer_manipulator");
}

MapManager::MapManager(const Config& config) : config_(config.checkValid()) {
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
    tickers_.emplace_back(
        config_.prune_active_blocks_frequency,
        [this](SubmapCollection* submaps) { pruneActiveBlocks(submaps); });
  }
  if (config_.activity_management_frequency > 0) {
    tickers_.emplace_back(
        config_.activity_management_frequency,
        [this](SubmapCollection* submaps) { manageSubmapActivity(submaps); });
  }
  if (config_.change_detection_frequency > 0) {
    tickers_.emplace_back(
        config_.change_detection_frequency,
        [this](SubmapCollection* submaps) { performChangeDetection(submaps); });
  }

  if (config_.prune_tracked_instances_frequency > 0) {
    tickers_.emplace_back(
        config_.prune_tracked_instances_frequency,
        [this](SubmapCollection* submaps) { pruneTrackedInstances(submaps); });
  }
}

void MapManager::tick(SubmapCollection* submaps) {
  // Increment counts for all tickers, which execute the requested actions.
  for (Ticker& ticker : tickers_) {
    ticker.tick(submaps);
  }
}

void MapManager::pruneActiveBlocks(SubmapCollection* submaps) {
  // Process all active instance and background submaps.
  auto t1 = std::chrono::high_resolution_clock::now();
  Timer timer("map_management/prune_active_blocks");
  std::stringstream info;
  std::vector<int> submaps_to_remove;
  for (Submap& submap : *submaps) {
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
    submaps->removeSubmap(id);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  timer.Stop();
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Pruned active blocks in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
      << "ms." << info.str();
}

void MapManager::manageSubmapActivity(SubmapCollection* submaps) {
  CHECK_NOTNULL(submaps);
  std::unordered_set<int> active_submaps;
  if (config_.merge_deactivated_submaps_if_possible) {
    // Track de-activated submaps if requested.
    for (const Submap& submap : *submaps) {
      if (submap.isActive()) {
        active_submaps.insert(submap.getID());
      }
    }
  }

  // Perform activity management.
  activity_manager_->processSubmaps(submaps);

  // Process de-activated submaps if requested.
  if (config_.merge_deactivated_submaps_if_possible ||
      config_.apply_class_layer_when_deactivating_submaps) {
    std::unordered_set<int> deactivated_submaps;
    for (Submap& submap : *submaps) {
      if (!submap.isActive() &&
          active_submaps.find(submap.getID()) != active_submaps.end()) {
        deactivated_submaps.insert(submap.getID());
      }
    }

    // Apply the class layer if requested.
    if (config_.apply_class_layer_when_deactivating_submaps) {
      for (int id : deactivated_submaps) {
        Submap* submap = submaps->getSubmapPtr(id);
        submap->applyClassLayer(*layer_manipulator_);
      }
    }

    // Try to merge the submaps.
    if (config_.merge_deactivated_submaps_if_possible) {
      for (int id : deactivated_submaps) {
        int merged_id;
        int current_id = id;
        while (mergeSubmapIfPossible(submaps, current_id, &merged_id)) {
          current_id = merged_id;
        }
        if (current_id == id) {
          LOG_IF(INFO, config_.verbosity >= 4)
              << "Submap " << id << " was deactivated, could not be matched."
              << std::endl;
        }
      }
    }
  }
}

void MapManager::performChangeDetection(SubmapCollection* submaps) {
  tsdf_registrator_->checkSubmapCollectionForChange(submaps);
}

void MapManager::finishMapping(SubmapCollection* submaps) {
  // Remove all empty blocks.
  std::stringstream info;
  info << "Finished mapping: ";
  for (Submap& submap : *submaps) {
    info << pruneBlocks(&submap);
  }
  LOG_IF(INFO, config_.verbosity >= 3) << info.str();

  // Deactivate last submaps.
  for (Submap& submap : *submaps) {
    if (submap.isActive()) {
      LOG_IF(INFO, config_.verbosity >= 3)
          << "Deactivating submap " << submap.getID();
      submap.finishActivePeriod();
    }
  }
  LOG_IF(INFO, config_.verbosity >= 3) << "Merging Submaps:";

  // Merge what is possible.
  bool merged_something = true;
  while (merged_something) {
    for (Submap& submap : *submaps) {
      merged_something = mergeSubmapIfPossible(submaps, submap.getID());
      if (merged_something) {
        break;
      }
    }
  }

  // Finish submaps.
  if (config_.apply_class_layer_when_deactivating_submaps) {
    LOG_IF(INFO, config_.verbosity >= 3) << "Applying class layers:";
    std::vector<int> empty_submaps;
    for (Submap& submap : *submaps) {
      if (submap.hasClassLayer()) {
        if (!submap.applyClassLayer(*layer_manipulator_)) {
          empty_submaps.emplace_back(submap.getID());
        }
      }
    }
    for (const int id : empty_submaps) {
      submaps->removeSubmap(id);
      LOG_IF(INFO, config_.verbosity >= 3)
          << "Removed submap " << id << " which was empty.";
    }
  }
}

bool MapManager::mergeSubmapIfPossible(SubmapCollection* submaps, int submap_id,
                                       int* merged_id) {
  // Use on inactive submaps, checks for possible matches with other inactive
  // submaps.
  if (!submaps->submapIdExists(submap_id)) {
    return false;
  }

  // Setup.
  Submap* submap = submaps->getSubmapPtr(submap_id);
  if (submap->isActive()) {
    // Active submaps need first to be de-activated.
    submap->finishActivePeriod();
  } else if (submap->getChangeState() == ChangeState::kAbsent) {
    return false;
  }

  // Find all potential matches.
  for (Submap& other : *submaps) {
    if (other.isActive() || other.getClassID() != submap->getClassID() ||
        other.getID() == submap->getID() ||
        !submap->getBoundingVolume().intersects(other.getBoundingVolume())) {
      continue;
    }

    bool submaps_match;
    if (!tsdf_registrator_->submapsConflict(*submap, other, &submaps_match)) {
      if (submaps_match) {
        // It's a match, merge the submap into the candidate.

        // Make sure both maps have or don't have class layers.
        if (!(submap->hasClassLayer() && other.hasClassLayer())) {
          submap->applyClassLayer(*layer_manipulator_);
          other.applyClassLayer(*layer_manipulator_);
        }
        layer_manipulator_->mergeSubmapAintoB(*submap, &other);
        LOG_IF(INFO, config_.verbosity >= 4)
            << "Merged Submap " << submap->getID() << " into " << other.getID()
            << ".";
        other.setChangeState(ChangeState::kPersistent);
        submaps->removeSubmap(submap_id);
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
    ClassBlock::Ptr class_block;
    if (class_layer) {
      if (class_layer->hasBlock(block_index)) {
        class_block = class_layer->getBlockPtrByIndex(block_index);
      }
    }
    const TsdfBlock& tsdf_block = tsdf_layer->getBlockByIndex(block_index);
    bool has_beloning_voxels = false;

    // Check all voxels.
    for (int voxel_index = 0; voxel_index < voxel_indices; ++voxel_index) {
      if (tsdf_block.getVoxelByLinearIndex(voxel_index).weight >= 1e-6) {
        if (class_block) {
          if (class_block->getVoxelByLinearIndex(voxel_index)
                  .belongsToSubmap()) {
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

void MapManager::pruneTrackedInstances(SubmapCollection* submaps) {
  CHECK_NOTNULL(submaps);

  if (!submaps->isSingleTsdf()) {
    LOG_IF(WARNING, config_.verbosity >= 4)
        << "Tracked instances pruning is only done in single TSDF mode!";
    return;
  }

  // Sanity check - there should be only one submap in single TSDF mode
  if (submaps->size() != 1) {
    return;
  }

  Submap& submap = *submaps->begin();

  if (!submap.hasClassLayer()) {
    LOG(WARNING) << "Tracked instance pruning requires class layer!";
    return;
  }
  ClassLayer* class_layer = submap.getClassLayerPtr().get();

  const int voxel_indices = std::pow(submap.getConfig().voxels_per_side, 3);
  std::unordered_map<int, size_t> instance_id_to_voxel_count;

  voxblox::BlockIndexList block_indices;
  class_layer->getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    const ClassBlock::Ptr class_block =
        class_layer->getBlockPtrByIndex(block_index);

    // Check all voxels.
    for (int voxel_index = 0; voxel_index < voxel_indices; ++voxel_index) {
      const ClassVoxel& voxel = class_block->getVoxelByLinearIndex(voxel_index);
      if (voxel.isObserverd() && voxel.getBelongingID() != 0) {
        instance_id_to_voxel_count[voxel.getBelongingID()] += 1;
      }
    }
  }

  // TODO(albanesg): this threshold is hardcoded and should be configurable
  size_t count = 0;
  const size_t min_instance_voxel_count =
      static_cast<int>(std::round(0.2 * submap.getConfig().voxel_size));
  for (const auto& [instance_id, voxel_count] : instance_id_to_voxel_count) {
    TrackedInstanceInfo* tracked_instance_info_ptr =
        submaps->getTrackedInstanceInfoPtr(instance_id);
    if (!tracked_instance_info_ptr) {
      submaps->removeTrackedInstanceInfo(instance_id);
      continue;
    }

    if (voxel_count < min_instance_voxel_count) {
      submaps->removeTrackedInstanceInfo(instance_id);
      count++;
    }
  }

  LOG_IF(INFO, config_.verbosity >= 4)
      << "Removed " << count << " tracked instances";
}

void MapManager::Ticker::tick(SubmapCollection* submaps) {
  // Perform 'action' every 'max_ticks' ticks.
  current_tick_++;
  if (current_tick_ >= max_ticks_) {
    action_(submaps);
    current_tick_ = 0;
  }
}

}  // namespace panoptic_mapping
