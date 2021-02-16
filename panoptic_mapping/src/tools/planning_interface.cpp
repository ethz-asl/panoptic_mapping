#include "panoptic_mapping/tools/planning_interface.h"

#include <limits>
#include <memory>
#include <utility>

#include <voxblox/interpolator/interpolator.h>

namespace panoptic_mapping {

PlanningInterface::PlanningInterface(
    std::shared_ptr<const SubmapCollection> submaps)
    : submaps_(std::move(submaps)) {}

bool PlanningInterface::isObserved(const Point& position,
                                   bool include_inactive_maps) const {
  for (const auto& submap : *submaps_) {
    if (include_inactive_maps || submap->isActive()) {
      const Point position_S = submap->getT_S_M() * position;
      if (submap->getBoundingVolume().contains_S(position_S)) {
        auto block_ptr =
            submap->getTsdfLayer().getBlockPtrByCoordinates(position_S);
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
  bool is_known_free = false;
  bool is_expected_free = false;
  bool is_expected_occupied = false;
  for (const auto& submap : *submaps_) {
    // Filter out irrelevant submaps.
    if (submap->getChangeState() == ChangeState::kAbsent) {
      continue;
    }
    if (submap->getLabel() == PanopticLabel::kFreeSpace) {
      if (is_known_free || (is_expected_free && !submap->isActive())) {
        continue;
      }
    } else if (!submap->isActive() && is_expected_occupied) {
      continue;
    }

    // Check the state.
    const Point position_S = submap->getT_S_M() * position;
    if (!submap->getBoundingVolume().contains_S(position_S)) {
      continue;
    }
    auto block_ptr =
        submap->getTsdfLayer().getBlockPtrByCoordinates(position_S);
    if (!block_ptr) {
      continue;
    }
    const TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(position_S);
    if (voxel.weight <= kObservedMinWeight_) {
      continue;
    }
    if (submap->getLabel() == PanopticLabel::kFreeSpace) {
      if (voxel.distance > 0.f) {
        if (submap->isActive()) {
          is_known_free = true;
        } else {
          is_expected_free = true;
        }
      }
    } else {
      if (voxel.distance <= 0.f) {
        if (submap->isActive()) {
          return VoxelState::kKnownOccupied;
        } else {
          is_expected_occupied = true;
        }
      } else {
        if (submap->isActive()) {
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
  } else if (is_expected_occupied) {
    return VoxelState::kExpectedOccupied;
  } else if (is_expected_free) {
    return VoxelState::kExpectedFree;
  }
  return VoxelState::kUnknown;
}

bool PlanningInterface::getDistance(const Point& position, float* distance,
                                    bool include_inactive_maps) const {
  // Get the Tsdf distance from the highest resolution submap covering the
  // target position. Return whether the point was observed.
  CHECK_NOTNULL(distance);
  float current_voxel_size = std::numeric_limits<float>::max();

  for (const auto& submap : *submaps_) {
    // Check activity.
    if (!submap->isActive() && !include_inactive_maps) {
      continue;
    }

    // Check resolution.
    if (submap->getTsdfLayer().voxel_size() >= current_voxel_size) {
      continue;
    }

    // Look up the voxel if it is observed.
    const Point position_S = submap->getT_S_M() * position;
    if (submap->getBoundingVolume().contains_S(position_S)) {
      voxblox::Interpolator<TsdfVoxel> interpolator(
          submap->getTsdfLayerPtr().get());
      if (interpolator.getDistance(position_S, distance, true)) {
        current_voxel_size = submap->getTsdfLayer().voxel_size();
      }
    }
  }
  return current_voxel_size < std::numeric_limits<float>::max();
}

}  // namespace panoptic_mapping
