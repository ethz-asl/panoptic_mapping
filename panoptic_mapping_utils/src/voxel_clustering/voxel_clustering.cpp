#include "panoptic_mapping_utils/voxel_clustering/voxel_clustering.h"

using IndexSet = voxblox::LongIndexSet;
using Point = voxblox::Point;
using Index = voxblox::GlobalIndex;

const Index kNeighborOffsets[26] = {
        Index(1, 0, 0),   Index(1, 1, 0),   Index(1, -1, 0),  Index(1, 0, 1),
        Index(1, 1, 1),   Index(1, -1, 1),  Index(1, 0, -1),  Index(1, 1, -1),
        Index(1, -1, -1), Index(0, 1, 0),   Index(0, -1, 0),  Index(0, 0, 1),
        Index(0, 1, 1),   Index(0, -1, 1),  Index(0, 0, -1),  Index(0, 1, -1),
        Index(0, -1, -1), Index(-1, 0, 0),  Index(-1, 1, 0),  Index(-1, -1, 0),
        Index(-1, 0, 1),  Index(-1, 1, 1),  Index(-1, -1, 1), Index(-1, 0, -1),
        Index(-1, 1, -1), Index(-1, -1, -1)};

bool panoptic_mapping::voxel_clustering::wavefrontExploration(
        const voxblox::GlobalIndex& seed, voxblox::LongIndexSet& closed_list,
        const panoptic_mapping::Submap* map, Instance* result) {
  // Setup search.
  Eigen::Matrix<voxblox::IndexElement, 3, 1> block_to_label;
  voxblox::VoxelIndex voxel_to_label;

  const panoptic_mapping::ClassVoxel* seed_voxel =
          map->getClassLayer().getVoxelPtrByGlobalIndex(seed);
  if(!seed_voxel) {

    return false;
  }

  bool use_binary_class =
          (seed_voxel->current_index == -1 && seed_voxel->counts.empty());
  if (!use_binary_class && seed_voxel->current_index == -1) {
    // No class assigned to this voxel. Should not happen
    return false;
  }
  int semantic_class = seed_voxel->current_index;

  if (use_binary_class) {
    semantic_class = classVoxelBelongsToSubmap(*seed_voxel);
  }

  std::stack<Index> open_stack;

  auto voxel_size = map->getConfig().voxel_size;
  CHECK_GT(voxel_size, 0.f);
  open_stack.push(seed);
  // Search all frontiers.
  while (!open_stack.empty()) {
    // 'current', including the initial point, traverse observed free space.
    Index current = open_stack.top();
    open_stack.pop();

    // Check all neighbors for frontiers and free space.
    for (const Index& offset : kNeighborOffsets) {
      Index candidate = current + offset;
      if (closed_list.find(candidate) != closed_list.end()) {
        // Only consider voxels that were not yet checked.
        continue;
      }
      closed_list.insert(candidate);

      voxblox::BlockIndex block_idx;
      voxblox::VoxelIndex voxel_idx;
      voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
              candidate, map->getClassLayer().voxels_per_side(), &block_idx,
              &voxel_idx);

      std::shared_ptr<const TsdfBlock> block =
              map->getTsdfLayer().getBlockPtrByIndex(block_idx);
      if (block) {
        const voxblox::TsdfVoxel& voxel =
                block->getVoxelByVoxelIndex(voxel_idx);
        if (voxel.weight > 1e-6) {
          if (voxel.distance > map->getTsdfLayer().voxel_size()) {
            // Note(schmluk): The surface is slightly inflated to make detection
            // more conservative and avoid frontiers out in the blue.
          } else {
            // Occupied voxel check class.
            const panoptic_mapping::ClassVoxel& c_voxel =
                    map->getClassLayer()
                            .getBlockPtrByIndex(block_idx)
                            ->getVoxelByVoxelIndex(voxel_idx);

            if ((!use_binary_class && c_voxel.current_index == semantic_class) ||
                (use_binary_class &&
                 semantic_class == classVoxelBelongsToSubmap(c_voxel))) {
              result->push_back(
                      voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
                              block_idx, voxel_idx,
                              map->getClassLayer().voxels_per_side()));
              open_stack.push(candidate);
            }
          }
        }
      }
    }
  }
  return true;
}

bool panoptic_mapping::voxel_clustering::getConnectedInstancesForSeeds(
        std::vector<voxblox::GlobalIndex>& seeds,
        const panoptic_mapping::Submap* map, std::vector<Instance>* instances) {
  // Stores all voxels that have been checked so far
  IndexSet all_checked_voxels;

  for (Index seed : seeds) {

    // Check if seed is allready part of an instance. If yes, don't use it
    if (all_checked_voxels.find(seed) != all_checked_voxels.end()) {
      // Only consider voxels as seed that were not yet checked.
      continue;
    }
    IndexSet closed_list;
    std::vector<voxblox::GlobalIndex> indexes_for_seed;

    if (wavefrontExploration(seed, closed_list, map, &indexes_for_seed)) {
      for (const auto& idx : indexes_for_seed) {
        all_checked_voxels.insert(idx);
      }
      if (!indexes_for_seed.empty()) {
        instances->push_back(indexes_for_seed);
      }
    } else {

//      return false;
    }
  }
  return true;
}
bool panoptic_mapping::voxel_clustering::getAllConnectedInstances(
        const panoptic_mapping::Submap* map, std::vector<Instance>* instances) {
  // Get all surface voxels
  std::vector<Index> surface_voxels;
  voxblox::BlockIndexList all_blocks;
  map->getTsdfLayer().getAllAllocatedBlocks(&all_blocks);
  const float voxel_size = map->getTsdfLayer().voxel_size();
  const int voxels_per_side = map->getTsdfLayer().voxels_per_side();

  for (auto block_idx : all_blocks) {
    auto block = map->getTsdfLayer().getBlockPtrByIndex(block_idx);
    for (int i = 0; i < block->num_voxels(); i++) {
      auto voxel = block->getVoxelByLinearIndex(i);
      if (voxel.distance <= voxel_size && voxel.distance >= -voxel_size) {
        auto voxel_idx =  voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
                block_idx, block->computeVoxelIndexFromLinearIndex(i),
                voxels_per_side);

        // Surface voxel
        auto c_voxel = map->getClassLayer().getVoxelPtrByGlobalIndex(voxel_idx);
        if (c_voxel->current_index != -1 || (c_voxel->belongs_count != 0 && c_voxel->foreign_count != 0))
          surface_voxels.push_back(voxel_idx);
      }
    }
  }

  return getConnectedInstancesForSeeds(surface_voxels, map, instances);
}

void panoptic_mapping::voxel_clustering::scoreConnectedInstances(
        const panoptic_mapping::Submap* map, const std::vector<Instance>* instances,
        bool include_gt, ScoringMethod scoring_method,
        std::priority_queue<InstanceInfo>* scored_instances) {
  for (Instance instance : *instances) {
    InstanceInfo info(0, 0.0f, -1);

    for (auto voxel_idx : instance) {
      const panoptic_mapping::ClassVoxelType* c_voxel =
              map->getClassLayer().getVoxelPtrByGlobalIndex(voxel_idx);

      if (info.size == 0) {
        // First voxel
        if (c_voxel->current_index == -1 && c_voxel->counts.empty()) {
          // Binary classification
          info.assigned_class = classVoxelBelongsToSubmap(*c_voxel);
        } else {
          info.assigned_class = c_voxel->current_index;
        }
      }

      info.instance.push_back(voxel_idx);

      if (!include_gt && c_voxel->is_groundtruth) continue;

      switch (scoring_method) {
        case UNCERTAINTY:
          info.mean_score += classVoxelUncertainty(*c_voxel);
              break;
        case BELONGS_PROBABILITY:
          info.mean_score += classVoxelBelongingProbability(*c_voxel);
              break;
        case ENTROPY:
          info.mean_score += classVoxelEntropy(*c_voxel);
              break;
        case SIZE:
          info.mean_score++;
              break;
      }
      info.size += 1;
    }
    if (scoring_method != ScoringMethod::SIZE) {
      // Calculate mean values
      if (info.size == 0)
        info.mean_score = 0;
      else
        info.mean_score = info.mean_score / info.size;
    }
    scored_instances->push(info);
  }
}