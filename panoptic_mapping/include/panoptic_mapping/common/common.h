#ifndef PANOPTIC_MAPPING_COMMON_COMMON_H_
#define PANOPTIC_MAPPING_COMMON_COMMON_H_

#include <glog/logging.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/utils/timing.h>

namespace panoptic_mapping {
// Type definitions to work with a voxblox map.
using FloatingPoint = voxblox::FloatingPoint;

using Point = voxblox::Point;
using Transformation = voxblox::Transformation;
using Pointcloud = voxblox::Pointcloud;

using TsdfVoxel = voxblox::TsdfVoxel;
using TsdfBlock = voxblox::Block<voxblox::TsdfVoxel>;
using TsdfLayer = voxblox::Layer<voxblox::TsdfVoxel>;

using Color = voxblox::Color;
using Colors = voxblox::Colors;

// Panoptic type labels.
enum class PanopticLabel { kUnknown = 0, kInstance, kBackground, kFreeSpace };

// Frame names are abreviated consistently: S - Submap, M - Mission, C - Camera

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_COMMON_H_
