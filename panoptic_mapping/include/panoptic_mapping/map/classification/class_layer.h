#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_LAYER_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_LAYER_H_

#include <memory>
#include <utility>

#include <voxblox/core/layer.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/class_block.h"
#include "panoptic_mapping/map/classification/class_voxel.h"

namespace panoptic_mapping {

/**
 * @brief General interface to classification voxel layers. Wraps the voxblox
 * layer to allow substituting different classification layer types.
 */
class ClassLayer {
 public:
  virtual ~ClassLayer() = default;

  virtual ClassVoxelType getVoxelType() const = 0;

  virtual ClassBlock::ConstPtr getBlockPtrByIndex(
      const BlockIndex& index) const = 0;
  virtual ClassBlock::Ptr getBlockPtrByIndex(const BlockIndex& index) = 0;
  virtual ClassBlock::Ptr allocateBlockPtrByIndex(const BlockIndex& index) = 0;
  virtual ClassBlock::ConstPtr getBlockPtrByCoordinates(
      const Point& coords) const = 0;
  virtual ClassBlock::Ptr getBlockPtrByCoordinates(const Point& coords) = 0;
  virtual ClassBlock::Ptr allocateBlockPtrByCoordinates(
      const Point& coords) = 0;
  virtual ClassBlock::Ptr allocateNewBlock(const BlockIndex& index) = 0;
  virtual ClassBlock::Ptr allocateNewBlockByCoordinates(
      const Point& coords) = 0;
  virtual void removeBlock(const BlockIndex& index) = 0;
  virtual void removeAllBlocks() = 0;
  virtual void removeBlockByCoordinates(const Point& coords) = 0;
  virtual void getAllAllocatedBlocks(voxblox::BlockIndexList* blocks) const = 0;
  virtual void getAllUpdatedBlocks(voxblox::Update::Status bit,
                                   voxblox::BlockIndexList* blocks) const = 0;
  virtual size_t getNumberOfAllocatedBlocks() const = 0;
  virtual bool hasBlock(const BlockIndex& block_index) const = 0;
  // TODO(schmluk): Check save/load functionality and only expose the needed
  // functions. virtual void getProto(voxblox::LayerProto* proto) const = 0;
  // virtual bool isCompatible(const voxblox::LayerProto& layer_proto) const =
  // 0; virtual bool isCompatible(const voxblox::BlockProto& layer_proto) const
  // = 0; virtual bool saveToFile(const std::string& file_path,
  //                         bool clear_file = true) const = 0;
  virtual bool saveBlocksToStream(bool include_all_blocks,
                                  voxblox::BlockIndexList blocks_to_include,
                                  std::fstream* outfile_ptr) const = 0;
  virtual bool addBlockFromProto(const voxblox::BlockProto& block_proto) = 0;
  virtual size_t getMemorySize() const = 0;
  virtual size_t voxels_per_side() const = 0;
  virtual FloatingPoint voxel_size() const = 0;
  virtual FloatingPoint block_size() const = 0;
  virtual std::unique_ptr<ClassLayer> clone() const = 0;

  /**
   * @brief Directly returns the voxel if it exists. Returns nullpointer
   * otherwise. Prefer this when looking up sparse voxels since it avoids the
   * ClassBlock intermediate step.
   */
  virtual ClassVoxel* getVoxelPtrByCoordinates(const Point& coords) = 0;
  virtual const ClassVoxel* getVoxelPtrByCoordinates(
      const Point& coords) const = 0;
};

/**
 * @brief Default implementation for the class layer wrapper which implements
 * the minimum set of functionalities.
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
  // void getProto(voxblox::LayerProto* proto) const override {layer_.
  // }
  // bool isCompatible(const voxblox::LayerProto& layer_proto) const override
  // {layer_.
  // }
  // bool isCompatible(const voxblox::BlockProto& layer_proto) const override
  // {layer_.
  // }
  // bool saveToFile(const std::string& file_path,
  //                         bool clear_file = true) const override {layer_.
  // }
  bool saveBlocksToStream(bool include_all_blocks,
                          voxblox::BlockIndexList blocks_to_include,
                          std::fstream* outfile_ptr) const override {
    // return layer_.saveBlocksToStream(include_all_blocks, blocks_to_include,
    //                                  outfile_ptr);
  }
  bool addBlockFromProto(const voxblox::BlockProto& block_proto) override {
    // return layer_.addBlockFromProto(
    //     block_proto, voxblox::Layer<VoxelT>::BlockMergingStrategy::kReplace);
  }
  size_t getMemorySize() const override { return layer_.getMemorySize(); }
  size_t voxels_per_side() const override { return layer_.voxels_per_side(); }
  FloatingPoint voxel_size() const override { return layer_.voxel_size(); }
  FloatingPoint block_size() const override { return layer_.block_size(); }

  ClassVoxel* getVoxelPtrByCoordinates(const Point& coords) override {
    return layer_.getVoxelPtrByCoordinates(coords);
  }
  const ClassVoxel* getVoxelPtrByCoordinates(
      const Point& coords) const override {
    return layer_.getVoxelPtrByCoordinates(coords);
  }

 protected:
  voxblox::Layer<VoxelT> layer_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_LAYER_H_
