#ifndef PANOPTIC_MAPPING_COMMON_COMMON_H_
#define PANOPTIC_MAPPING_COMMON_COMMON_H_

#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>

#include <glog/logging.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox/utils/timing.h>

namespace panoptic_mapping {
/**
 * @brief Common Type definitions for the full framework.
 */

// Types.
// Type used for counting voxels. This stores up to ~65k measurements so should
// never run out. If this parameter is changed double check that all
// serialization still works!
using ClassificationCount = uint16_t;

// Wroking with voxblox maps.
using FloatingPoint = voxblox::FloatingPoint;
using VoxelIndex = voxblox::VoxelIndex;
using BlockIndex = voxblox::BlockIndex;
using Color = voxblox::Color;

// Geometry.
using Point = voxblox::Point;
using Transformation = voxblox::Transformation;
using Pointcloud = voxblox::Pointcloud;

// Tsdf and mesh Maps. Classification maps are defined in class_layer.h
using TsdfVoxel = voxblox::TsdfVoxel;
using TsdfBlock = voxblox::Block<TsdfVoxel>;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using MeshLayer = voxblox::MeshLayer;

// Panoptic type labels.
enum class PanopticLabel { kUnknown = 0, kInstance, kBackground, kFreeSpace };
inline std::string panopticLabelToString(const PanopticLabel& label) {
  switch (label) {
    case PanopticLabel::kUnknown:
      return "Unknown";
    case PanopticLabel::kInstance:
      return "Instance";
    case PanopticLabel::kBackground:
      return "Background";
    case PanopticLabel::kFreeSpace:
      return "FreeSpace";
  }
}

// Iso-surface-points are used to check alignment and represent the surface
// of finished submaps.
struct IsoSurfacePoint {
  IsoSurfacePoint(Point _position, FloatingPoint _weight)
      : position(std::move(_position)), weight(_weight) {}
  Point position;
  FloatingPoint weight;
};

// Change detection data stores relevant information for associating submaps.
enum class ChangeState {
  kNew = 0,
  kMatched,
  kUnobserved,
  kAbsent,
  kPersistent
};

inline std::string changeStateToString(const ChangeState& state) {
  switch (state) {
    case ChangeState::kNew:
      return "New";
    case ChangeState::kMatched:
      return "Mathced";
    case ChangeState::kPersistent:
      return "Persistent";
    case ChangeState::kAbsent:
      return "Absent";
    case ChangeState::kUnobserved:
      return "Unobserved";
    default:
      return "UnknownChangeState";
  }
}

/**
 * Frame names are abbreviated consistently (in paranthesesalternative
 * explanations):
 * S - Submap
 * M - Mission (Map / World)
 * C - Camera (Sensor)
 */

// Timing.
#define PANOPTIC_MAPPING_TIMING_ENABLED  // Unset to disable all timers.
#ifdef PANOPTIC_MAPPING_TIMING_ENABLED
using Timer = voxblox::timing::Timer;
#else
using Timer = voxblox::timing::DummyTimer;
#endif  // PANOPTIC_MAPPING_TIMING_ENABLED
using Timing = voxblox::timing::Timing;

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_COMMON_H_
