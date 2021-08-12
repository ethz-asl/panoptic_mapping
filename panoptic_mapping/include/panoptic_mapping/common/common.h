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

#include "panoptic_mapping/common/class_voxel.h"

namespace panoptic_mapping {
// Type definitions to work with a voxblox map.
using FloatingPoint = voxblox::FloatingPoint;

// Geometry.
using Point = voxblox::Point;
using Transformation = voxblox::Transformation;
using Pointcloud = voxblox::Pointcloud;

// Tsdf and class Maps.
using TsdfVoxel = voxblox::TsdfVoxel;
using TsdfBlock = voxblox::Block<TsdfVoxel>;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using MeshLayer = voxblox::MeshLayer;
using ClassBlock = voxblox::Block<ClassVoxel>;
using ClassLayer = voxblox::Layer<ClassVoxel>;

using Color = voxblox::Color;

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
