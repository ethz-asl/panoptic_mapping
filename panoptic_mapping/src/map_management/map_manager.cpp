#include "panoptic_mapping/map_management/map_manager.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace panoptic_mapping {

void MapManager::Config::checkParams() const {
  checkParamConfig(activity_manager_config);
  checkParamConfig(tsdf_registrator_config);
}

void MapManager::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);

  setupParam("prune_active_blocks_frequency", &prune_active_blocks_frequency);
  setupParam("activity_management_frequency", &activity_management_frequency);
  setupParam("change_detection_frequency", &change_detection_frequency);

  setupParam("activity_manager_config", &activity_manager_config,
             "activity_manager");
  setupParam("tsdf_registrator_config", &tsdf_registrator_config,
             "tsdf_registrator");
}

MapManager::MapManager(const Config& config,
                       std::shared_ptr<SubmapCollection> map)
    : config_(config.checkValid()),
      map_(std::move(map)),
      activity_manager_(config_.activity_manager_config),
      tsdf_registrator_(config_.tsdf_registrator_config) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Add all requested tasks.
  if (config_.prune_active_blocks_frequency > 0) {
    tickers_.emplace_back(config_.prune_active_blocks_frequency,
                          [this] { pruneActiveBlocks(); });
  }
  if (config_.activity_management_frequency > 0) {
    tickers_.emplace_back(config_.activity_management_frequency, [this] {
      activity_manager_.processSubmaps(map_.get());
    });
  }
  if (config_.change_detection_frequency > 0) {
    tickers_.emplace_back(config_.change_detection_frequency,
                          [this] { performChangeDetection(); });
  }
}

void MapManager::tickMapManagement() {
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
  for (auto& submap_ptr : *map_) {
    if (submap_ptr->getLabel() == PanopticLabel::kFreeSpace ||
        !submap_ptr->isActive()) {
      continue;
    }
    info << pruneBlocks(submap_ptr.get());

    // If a submap does not contain data anymore it can be removed.
    if (submap_ptr->getTsdfLayer().getNumberOfAllocatedBlocks() == 0) {
      submaps_to_remove.emplace_back(submap_ptr->getID());
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

void MapManager::performChangeDetection() {
  tsdf_registrator_.checkSubmapCollectionForChange(map_.get());
}

std::string MapManager::pruneBlocks(Submap* submap) {
  // Only process maps with class data.
  if (!submap->getConfig().use_class_layer) {
    return "";
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  // Setup.
  const std::shared_ptr<ClassLayer>& class_layer = submap->getClassLayerPtr();
  const std::shared_ptr<TsdfLayer>& tsdf_layer = submap->getTsdfLayerPtr();
  const std::shared_ptr<MeshLayer>& mesh_layer = submap->getMeshLayerPtr();
  const int voxel_indices = std::pow(submap->getConfig().voxels_per_side, 3);
  int count = 0;

  // Remove all blocks that don't have any belonging voxels.
  voxblox::BlockIndexList block_indices;
  class_layer->getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    ClassBlock& class_block = class_layer->getBlockByIndex(block_index);
    bool has_beloning_voxels = false;

    // Check all voxels.
    for (int voxel_index = 0; voxel_index < voxel_indices; ++voxel_index) {
      if (class_block.getVoxelByLinearIndex(voxel_index).belongsToSubmap()) {
        has_beloning_voxels = true;
        break;
      }
    }

    // Prune blocks.
    if (!has_beloning_voxels) {
      class_layer->removeBlock(block_index);
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
