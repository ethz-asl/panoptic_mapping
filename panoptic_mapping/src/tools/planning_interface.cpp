#include "panoptic_mapping/tools/planning_interface.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <voxblox/interpolator/interpolator.h>

namespace panoptic_mapping {

PlanningInterface::PlanningInterface(
    std::shared_ptr<const SubmapCollection> submaps)
    : submaps_(std::move(submaps)) {}

bool PlanningInterface::isObserved(const Point& position,
                                   bool include_inactive_maps) const {
  Timer timer("planning_interface/is_observed");
  // TODO(schmluk): Update this to latest convetions.
  for (const Submap& submap : *submaps_) {
    if (include_inactive_maps || submap.isActive()) {
      const Point position_S = submap.getT_S_M() * position;
      if (submap.getBoundingVolume().contains_S(position_S)) {
        auto block_ptr =
            submap.getTsdfLayer().getBlockPtrByCoordinates(position_S);
        if (block_ptr) {
          const TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(position_S);
          if (voxel.weight >= kObservedMinWeight_) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

PlanningInterface::VoxelState PlanningInterface::getVoxelState(
    const Point& position) const {
  Timer timer("planning_interface/get_voxel_state");
  bool is_known_free = false;
  bool is_expected_free = false;
  bool is_expected_occupied = false;
  bool is_persistent_occupied = false;
  for (const auto& submap : *submaps_) {
    // Filter out irrelevant submaps.
    if (submap.getChangeState() == ChangeState::kAbsent) {
      continue;
    }
    if (submap.getLabel() == PanopticLabel::kFreeSpace) {
      if (is_known_free || (is_expected_free && !submap.isActive())) {
        continue;
      }
    } else if (!submap.isActive()) {
      if (is_persistent_occupied) {
        continue;
      }
      if (submap.getChangeState() == ChangeState::kUnobserved &&
          is_expected_occupied) {
        continue;
      }
    }

    // Check the state.
    const Point position_S = submap.getT_S_M() * position;
    if (!submap.getBoundingVolume().contains_S(position_S)) {
      continue;
    }
    auto block_ptr = submap.getTsdfLayer().getBlockPtrByCoordinates(position_S);
    if (!block_ptr) {
      continue;
    }
    const TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(position_S);
    if (voxel.weight <= kObservedMinWeight_) {
      continue;
    }
    const float voxel_size = submap.getConfig().voxel_size;
    if (submap.getLabel() == PanopticLabel::kFreeSpace) {
      if (voxel.distance > voxel_size) {
        if (submap.isActive()) {
          is_known_free = true;
        } else {
          is_expected_free = true;
        }
      }
    } else {
      if (voxel.distance <= voxel_size) {
        if (submap.isActive()) {
          return VoxelState::kKnownOccupied;
        } else if (submap.getChangeState() == ChangeState::kPersistent) {
          is_persistent_occupied = true;
        } else if (submap.getChangeState() == ChangeState::kUnobserved) {
          is_expected_occupied = true;
        }
      } else {
        if (submap.isActive()) {
          is_known_free = true;
        } else {
          is_expected_free = true;
        }
      }
    }
  }

  // Aggregate result.
  if (is_known_free) {
    return VoxelState::kKnownFree;
  } else if (is_persistent_occupied) {
    return VoxelState::kPersistentOccupied;
  } else if (is_expected_occupied) {
    return VoxelState::kExpectedOccupied;
  } else if (is_expected_free) {
    return VoxelState::kExpectedFree;
  }
  return VoxelState::kUnknown;
}  // namespace panoptic_mapping

bool PlanningInterface::getDistance(const Point& position, float* distance,
                                    bool consider_change_state,
                                    bool include_free_space) const {
  Timer timer("planning_interface/get_distance");
  // Get the Tsdf distance. Return whether the point was observed.
  CHECK_NOTNULL(distance);
  constexpr float max = std::numeric_limits<float>::max();
  // Distances and observedness in order of priority: [active obj (max res),
  // persistent obj (min sdf), free space (fallback)]
  std::vector<float> current_distance(3, max);
  std::vector<bool> observed(3, false);
  float current_resolution = max;

  for (const auto& submap : *submaps_) {
    // Only include submaps considered present.
    if (consider_change_state &&
        (submap.getChangeState() == ChangeState::kAbsent ||
         submap.getChangeState() == ChangeState::kUnobserved)) {
      continue;
    }

    // Check priority ordering to avoid duplicate lookups.
    const bool is_free_space = submap.getLabel() == PanopticLabel::kFreeSpace;
    if (is_free_space && !include_free_space) {
      continue;
    }

    if (is_free_space) {
      if (observed[0] || observed[1]) {
        // The point was already observed by an object map so ignore freespace.
        continue;
      }
    } else {
      if (submap.isActive()) {
        if (submap.getConfig().voxel_size >= current_resolution) {
          // The point was already observed by a higher resolution active map.
          continue;
        }
      } else {
        if (observed[0]) {
          // The point was already observed by an active map.
          continue;
        }
      }
    }

    // Look up the voxel if it is observed.
    const Point position_S = submap.getT_S_M() * position;
    if (submap.getBoundingVolume().contains_S(position_S)) {
      // Check classification for inactive submaps.
      if (submap.hasClassLayer() && !submap.isActive()) {
        const ClassVoxel* class_voxel =
            submap.getClassLayer().getVoxelPtrByCoordinates(position_S);
        if (class_voxel) {
          if (class_voxel->belongsToSubmap()) {
            continue;
          }
        }
      }
      float sdf;
      voxblox::Interpolator<TsdfVoxel> interpolator(&(submap.getTsdfLayer()));
      if (interpolator.getDistance(position_S, &sdf, true)) {
        if (is_free_space) {
          current_distance[2] = std::min(current_distance[2], sdf);
          observed[2] = true;
        } else if (submap.isActive()) {
          // Active submaps reconstruct everything, take highest resolution
          // observation. Lower resolution observations are filtered before.
          current_distance[0] = sdf;
          current_resolution = submap.getConfig().voxel_size;
          observed[0] = true;
        } else {
          // Inactive submaps capture only their own geometry, return min.
          current_distance[1] = std::min(current_distance[1], sdf);
          observed[1] = true;
        }
      }
    }
  }

  // Aggregate result.
  for (size_t i = 0; i < 3; ++i) {
    if (observed[i]) {
      *distance = current_distance[i];
      return true;
    }
  }
  return false;
}

}  // namespace panoptic_mapping
