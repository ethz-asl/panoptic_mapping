#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_LAYER_IMPL_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_LAYER_IMPL_H_

#include <memory>
#include <utility>
#include <vector>

#include <voxblox/core/layer.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/class_block.h"
#include "panoptic_mapping/map/classification/class_voxel.h"
#include "panoptic_mapping/tools/serialization.h"

namespace panoptic_mapping {

/**
 * @brief Default implementation for the class layer wrapper which implements
 * the shared set of functionalities (voxblox interfaces and serialization).
 *
 * @tparam VoxelT Voxel type to be stored in this layer.
 */
template <typename VoxelT>
class ClassLayerImpl : public ClassLayer {
 public:
  explicit ClassLayerImpl<VoxelT>(voxblox::FloatingPoint voxel_size,
                                  size_t voxels_per_side)
      : layer_(voxel_size, voxels_per_side) {}

  // Block access interfaces.
  ClassBlock::ConstPtr getBlockConstPtrByIndex(
      const BlockIndex& index) const override {
    return getClassBlockConstPtr(layer_.getBlockPtrByIndex(index));
  }
  ClassBlock::Ptr getBlockPtrByIndex(const BlockIndex& index) override {
    return getClassBlockPtr(layer_.getBlockPtrByIndex(index));
  }
  ClassBlock::Ptr allocateBlockPtrByIndex(const BlockIndex& index) override {
    return getClassBlockPtr(layer_.allocateBlockPtrByIndex(index));
  }
  ClassBlock::ConstPtr getBlockPtrByCoordinates(
      const Point& coords) const override {
    return getClassBlockConstPtr(layer_.getBlockPtrByCoordinates(coords));
  }
  ClassBlock::Ptr getBlockPtrByCoordinates(const Point& coords) override {
    return getClassBlockPtr(layer_.getBlockPtrByCoordinates(coords));
  }
  ClassBlock::Ptr allocateBlockPtrByCoordinates(const Point& coords) override {
    return getClassBlockPtr(layer_.allocateBlockPtrByCoordinates(coords));
  }
  ClassBlock::Ptr allocateNewBlock(const BlockIndex& index) override {
    return getClassBlockPtr(layer_.allocateNewBlock(index));
  }
  ClassBlock::Ptr allocateNewBlockByCoordinates(const Point& coords) override {
    return getClassBlockPtr(layer_.allocateNewBlockByCoordinates(coords));
  }

  // General functions.
  void removeBlock(const BlockIndex& index) override {
    layer_.removeBlock(index);
  }
  void removeAllBlocks() override { layer_.removeAllBlocks(); }
  void removeBlockByCoordinates(const Point& coords) override {
    layer_.removeBlockByCoordinates(coords);
  }
  void getAllAllocatedBlocks(voxblox::BlockIndexList* blocks) const override {
    layer_.getAllAllocatedBlocks(blocks);
  }
  void getAllUpdatedBlocks(voxblox::Update::Status bit,
                           voxblox::BlockIndexList* blocks) const override {
    layer_.getAllUpdatedBlocks(bit, blocks);
  }
  size_t getNumberOfAllocatedBlocks() const override {
    return layer_.getNumberOfAllocatedBlocks();
  }
  bool hasBlock(const BlockIndex& block_index) const override {
    return layer_.hasBlock(block_index);
  }

  size_t getMemorySize() const override { return layer_.getMemorySize(); }
  size_t voxels_per_side() const override { return layer_.voxels_per_side(); }
  FloatingPoint voxel_size() const override { return layer_.voxel_size(); }
  FloatingPoint block_size() const override { return layer_.block_size(); }

  // Serialization.
  bool saveBlockToStream(BlockIndex block_index,
                         std::fstream* outfile_ptr) const override {
    CHECK_NOTNULL(outfile_ptr);
    auto block = layer_.getBlockPtrByIndex(block_index);
    if (!block) {
      return false;
    }

    // Save data.
    voxblox::BlockProto proto;
    proto.set_has_data(block->has_data());
    proto.set_voxels_per_side(block->voxels_per_side());
    proto.set_voxel_size(block->voxel_size());
    proto.set_origin_x(block->origin().x());
    proto.set_origin_y(block->origin().y());
    proto.set_origin_z(block->origin().z());
    for (size_t i = 0; i < block->num_voxels(); ++i) {
      for (uint32_t word :
           reinterpret_cast<const ClassVoxel&>(block->getVoxelByLinearIndex(i))
               .serializeVoxelToInt()) {
        proto.add_voxel_data(word);
      }
    }
    if (!voxblox::utils::writeProtoMsgToStream(proto, outfile_ptr)) {
      LOG(ERROR) << "Could not write class block proto message to stream.";
      return false;
    }
    return true;
  }

  bool saveBlocksToStream(bool include_all_blocks,
                          voxblox::BlockIndexList blocks_to_include,
                          std::fstream* outfile_ptr) const override {
    CHECK_NOTNULL(outfile_ptr);
    // Get blocks to write.
    if (include_all_blocks) {
      layer_.getAllAllocatedBlocks(&blocks_to_include);
    }

    // Write blocks.
    for (const BlockIndex& index : blocks_to_include) {
      if (!saveBlockToStream(index, outfile_ptr)) {
        LOG(WARNING) << "Could not save block " << index.transpose()
                     << " to stream.";
      }
    }
    return true;
  }

  bool addBlockFromProto(const voxblox::BlockProto& block_proto) override {
    // Check compatibility.
    if (!isCompatible(block_proto, *this)) {
      return false;
    }

    // Add (potentially replace) the block.
    const Point origin(block_proto.origin_x(), block_proto.origin_y(),
                       block_proto.origin_z());
    layer_.removeBlockByCoordinates(origin);
    auto block = layer_.allocateNewBlockByCoordinates(origin);

    // Read the data.
    std::vector<uint32_t> data;
    data.resize(block_proto.voxel_data_size());
    size_t index = 0;
    for (uint32_t word : block_proto.voxel_data()) {
      data[index] = word;
      index++;
    }

    // Load the voxels.
    index = 0;
    for (size_t i = 0; i < block->num_voxels(); ++i) {
      if (!reinterpret_cast<ClassVoxel&>(block->getVoxelByLinearIndex(i))
               .deseriliazeVoxelFromInt(data, &index)) {
        LOG(WARNING) << "Could not serialize voxel from data.";
        return false;
      }
    }
    return true;
  }

  // Lookup.
  ClassVoxel* getVoxelPtrByCoordinates(const Point& coords) override {
    return layer_.getVoxelPtrByCoordinates(coords);
  }
  const ClassVoxel* getVoxelPtrByCoordinates(
      const Point& coords) const override {
    return layer_.getVoxelPtrByCoordinates(coords);
  }

  // Exposes the layer block if the type is known.
  voxblox::Layer<VoxelT>& getLayer() { return layer_; }
  const voxblox::Layer<VoxelT>& getLayer() const { return layer_; }

 protected:
  voxblox::Layer<VoxelT> layer_;

  // Common conversion functions for the class block wrapper.
  ClassBlock::Ptr getClassBlockPtr(
      const typename voxblox::Block<VoxelT>::Ptr& block_ptr) {
    if (block_ptr) {
      return ClassBlock::Ptr(new ClassBlockImpl<VoxelT>(block_ptr));
    }
    return ClassBlock::Ptr();
  }
  ClassBlock::ConstPtr getClassBlockConstPtr(
      const typename voxblox::Block<VoxelT>::ConstPtr& block_ptr) const {
    // NOTE(schmluk): Since const pointers are returned the const-ness is cast
    // away to create the ClassBlock Wrapper. The block itself should thus
    // still be protected.
    if (block_ptr) {
      return ClassBlock::ConstPtr(new ClassBlockImpl<VoxelT>(
          std::const_pointer_cast<voxblox::Block<VoxelT>>(block_ptr)));
    }
    return ClassBlock::ConstPtr();
  }
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_LAYER_IMPL_H_
