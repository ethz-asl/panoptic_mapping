#ifndef PANOPTIC_MAPPING_CORE_COMMON_H_
#define PANOPTIC_MAPPING_CORE_COMMON_H_

#include <glog/logging.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/utils/timing.h>

namespace panoptic_mapping {
// Taking some voxblox datatypes
using voxblox::Block;
using voxblox::Color;
using voxblox::Colors;
using voxblox::EsdfVoxel;
using voxblox::Layer;
using voxblox::Point;
using voxblox::Pointcloud;
using voxblox::Transformation;
using voxblox::TsdfVoxel;

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_COMMON_H_
