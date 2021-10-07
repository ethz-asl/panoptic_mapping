#include "panoptic_mapping/common/class_voxel_layer.h"

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/tools/serialization.h"

namespace panoptic_mapping {

template <>
bool ClassVoxelLayer<ClassUncertaintyVoxel>::saveBlocksToStream(
    bool include_all_blocks, voxblox::BlockIndexList blocks_to_include,
    std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  for (const BlockMapPair& pair : block_map_) {
    bool write_block_to_file = include_all_blocks;
    if (!write_block_to_file) {
      voxblox::BlockIndexList::const_iterator it = std::find(
          blocks_to_include.begin(), blocks_to_include.end(), pair.first);
      if (it != blocks_to_include.end()) {
        write_block_to_file = true;
      }
    }
    if (write_block_to_file) {
      voxblox::BlockProto block_proto;
      voxblox::getProtoForBlock<ClassUncertaintyVoxel>(
          pair.second.get(), &block_proto, serialize_top_n_classes_);

      if (!voxblox::utils::writeProtoMsgToStream(block_proto, outfile_ptr)) {
        LOG(ERROR) << "Could not write block message.";
        return false;
      }
    }
  }
  return true;
}

}  // namespace panoptic_mapping
