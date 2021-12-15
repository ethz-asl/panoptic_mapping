#include "panoptic_mapping/tools/serialization.h"

#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

#include <voxblox/Block.pb.h>
#include <voxblox/io/layer_io.h>

#include "panoptic_mapping/map/classification/binary_count.h"
#include "panoptic_mapping/map/classification/fixed_count.h"
#include "panoptic_mapping/map/classification/moving_binary_count.h"
#include "panoptic_mapping/map/classification/uncertainty.h"
#include "panoptic_mapping/map/classification/variable_count.h"

namespace panoptic_mapping {

std::unique_ptr<ClassLayer> loadClassLayerFromStream(
    const SubmapProto& submap_proto, std::istream* proto_file_ptr,
    uint64_t* tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);

  std::unique_ptr<ClassLayer> result;

  // Get the type of voxel needed.
  const ClassVoxelType voxel_type =
      static_cast<ClassVoxelType>(submap_proto.class_voxel_type());
  switch (voxel_type) {
    case ClassVoxelType::kBinaryCount: {
      result = BinaryCountLayer::loadFromStream(submap_proto, proto_file_ptr,
                                                tmp_byte_offset_ptr);
      break;
    }
    case ClassVoxelType::kMovingBinaryCount: {
      result = MovingBinaryCountLayer::loadFromStream(
          submap_proto, proto_file_ptr, tmp_byte_offset_ptr);
      break;
    }
    case ClassVoxelType::kFixedCount: {
      result = FixedCountLayer::loadFromStream(submap_proto, proto_file_ptr,
                                               tmp_byte_offset_ptr);
      break;
    }
    case ClassVoxelType::kVariableCount: {
      result = VariableCountLayer::loadFromStream(submap_proto, proto_file_ptr,
                                                  tmp_byte_offset_ptr);
      break;
    }
    case ClassVoxelType::kUncertainty: {
      result = UncertaintyLayer::loadFromStream(submap_proto, proto_file_ptr,
                                                tmp_byte_offset_ptr);
      break;
    }
  }
  if (!result) {
    LOG(ERROR) << "Couldn't read class layer type from stream.";
    return nullptr;
  }

  // Load the blocks.
  if (!loadClassBlocksFromStream(submap_proto, proto_file_ptr,
                                 tmp_byte_offset_ptr, result.get())) {
    LOG(ERROR) << "Could not read class blocks from stream.";
    return nullptr;
  }

  return result;
}

bool saveClassLayerToStream(const ClassLayer& layer) {
  // Currently just a dummy since not needed...
  return false;
}

bool loadClassBlocksFromStream(const SubmapProto& submap_proto,
                               std::istream* proto_file_ptr,
                               uint64_t* tmp_byte_offset_ptr,
                               ClassLayer* layer) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);
  CHECK_NOTNULL(layer);
  // Read all blocks and add them to the layer.
  for (uint32_t block_idx = 0u; block_idx < submap_proto.num_class_blocks();
       ++block_idx) {
    voxblox::BlockProto block_proto;
    if (!voxblox::utils::readProtoMsgFromStream(proto_file_ptr, &block_proto,
                                                tmp_byte_offset_ptr)) {
      LOG(ERROR) << "Could not read block protobuf message number "
                 << block_idx;
      return false;
    }

    if (!layer->addBlockFromProto(block_proto)) {
      LOG(ERROR)
          << "Could not add the block protobuf message to the class layer!";
      return false;
    }
  }
  return true;
}

bool isCompatible(const voxblox::BlockProto& block_proto,
                  const ClassLayer& layer) {
  if (std::fabs(block_proto.voxel_size() - layer.voxel_size()) >
      std::numeric_limits<FloatingPoint>::epsilon()) {
    LOG(ERROR) << "Class block proto to be loaded has incompatible 'voxel "
                  "size' with the layer: "
               << block_proto.voxel_size() << " vs. " << layer.voxel_size();
    return false;
  }
  if (block_proto.voxels_per_side() !=
      static_cast<int>(layer.voxels_per_side())) {
    LOG(ERROR) << "Class block proto to be loaded has incompatible 'voxels "
                  "per side' with the layer: "
               << block_proto.voxels_per_side() << " vs. "
               << layer.voxels_per_side();
    return false;
  }
  return true;
}

}  // namespace panoptic_mapping
