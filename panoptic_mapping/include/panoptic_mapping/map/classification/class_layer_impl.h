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

  // NOTE(schmluk): Since const pointers are returned the const-ness is cast
  // away to create the ClassBlock Wrapper. The block itself should thus still
  // be protected.
  ClassBlock::ConstPtr getBlockPtrByIndex(
      const BlockIndex& index) const override {
    auto block = layer_.getBlockPtrByIndex(index);
    if (block) {
      return std::make_shared<const ClassBlockImpl<VoxelT>>(
          std::const_pointer_cast<voxblox::Block<VoxelT>>(block));
    }
    return nullptr;
  }
  ClassBlock::Ptr getBlockPtrByIndex(const BlockIndex& index) override {
    auto block = layer_.getBlockPtrByIndex(index);
    if (block) {
      return std::make_shared<ClassBlockImpl<VoxelT>>(block);
    }
    return nullptr;
  }
  ClassBlock::Ptr allocateBlockPtrByIndex(const BlockIndex& index) override {
    auto block = layer_.allocateBlockPtrByIndex(index);
    return std::make_shared<ClassBlockImpl<VoxelT>>(block);
  }
  ClassBlock::ConstPtr getBlockPtrByCoordinates(
      const Point& coords) const override {
    auto block = layer_.getBlockPtrByCoordinates(coords);
    if (block) {
      return std::make_shared<const ClassBlockImpl<VoxelT>>(
          std::const_pointer_cast<voxblox::Block<VoxelT>>(block));
    }
    return nullptr;
  }
  ClassBlock::Ptr getBlockPtrByCoordinates(const Point& coords) override {
    auto block = layer_.getBlockPtrByCoordinates(coords);
    if (block) {
      return std::make_shared<ClassBlockImpl<VoxelT>>(block);
    }
    return nullptr;
  }
  ClassBlock::Ptr allocateBlockPtrByCoordinates(const Point& coords) override {
    auto block = layer_.allocateBlockPtrByCoordinates(coords);
    return std::make_shared<ClassBlockImpl<VoxelT>>(block);
  }
  ClassBlock::Ptr allocateNewBlock(const BlockIndex& index) override {
    auto block = layer_.allocateNewBlock(index);
    return std::make_shared<ClassBlockImpl<VoxelT>>(block);
  }
  ClassBlock::Ptr allocateNewBlockByCoordinates(const Point& coords) override {
    auto block = layer_.allocateNewBlockByCoordinates(coords);
    return std::make_shared<ClassBlockImpl<VoxelT>>(block);
  }
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
      auto block = layer_.getBlockPtrByIndex(index);
      if (!block) {
        continue;
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
        for (uint32_t word : reinterpret_cast<const ClassVoxel&>(
                                 block->getVoxelByLinearIndex(i))
                                 .serializeVoxelToInt())
          proto.add_voxel_data(word);
      }
      if (!voxblox::utils::writeProtoMsgToStream(proto, outfile_ptr)) {
        LOG(ERROR) << "Could not write class block proto message to stream.";
        return false;
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

    if (block_proto.has_data()) {
      // Read the data.
      std::vector<uint32_t> data;
      data.resize(block_proto.voxel_data_size());
      size_t index = 0;
      for (uint32_t word : block_proto.voxel_data()) {
        data[index++] = word;
      }

      // Load the voxels.
      index = 0;
      for (size_t i = 0; i < block->num_voxels(); ++i) {
        reinterpret_cast<ClassVoxel&>(block->getVoxelByLinearIndex(i))
            .deseriliazeVoxelFromInt(data, &index);
      }
    }
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
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_LAYER_IMPL_H_
