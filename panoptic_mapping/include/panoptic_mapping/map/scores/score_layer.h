#ifndef PANOPTIC_MAPPING_MAP_SCORES_SCORE_LAYER_H_
#define PANOPTIC_MAPPING_MAP_SCORES_SCORE_LAYER_H_

#include <memory>
#include <utility>

#include <voxblox/core/layer.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/scores/score_block.h"
#include "panoptic_mapping/map/scores/score_voxel.h"

namespace panoptic_mapping {

/**
 * @brief General interface to score voxel layers. Wraps the voxblox
 * layer to allow substituting different score layer types.
 */
class ScoreLayer {
 public:
  virtual ~ScoreLayer() = default;

  // Implement this to expose the voxel type to objects that use a specific
  // voxel type.
  virtual ScoreVoxelType getVoxelType() const = 0;

  // Voxblox interfaces.
  virtual ScoreBlock::ConstPtr getBlockConstPtrByIndex(
      const BlockIndex& index) const = 0;
  virtual ScoreBlock::Ptr getBlockPtrByIndex(const BlockIndex& index) = 0;
  virtual ScoreBlock::Ptr allocateBlockPtrByIndex(const BlockIndex& index) = 0;
  virtual ScoreBlock::ConstPtr getBlockPtrByCoordinates(
      const Point& coords) const = 0;
  virtual ScoreBlock::Ptr getBlockPtrByCoordinates(const Point& coords) = 0;
  virtual ScoreBlock::Ptr allocateBlockPtrByCoordinates(
      const Point& coords) = 0;
  virtual ScoreBlock::Ptr allocateNewBlock(const BlockIndex& index) = 0;
  virtual ScoreBlock::Ptr allocateNewBlockByCoordinates(
      const Point& coords) = 0;
  virtual void removeBlock(const BlockIndex& index) = 0;
  virtual void removeAllBlocks() = 0;
  virtual void removeBlockByCoordinates(const Point& coords) = 0;
  virtual void getAllAllocatedBlocks(voxblox::BlockIndexList* blocks) const = 0;
  virtual void getAllUpdatedBlocks(voxblox::Update::Status bit,
                                   voxblox::BlockIndexList* blocks) const = 0;
  virtual size_t getNumberOfAllocatedBlocks() const = 0;
  virtual bool hasBlock(const BlockIndex& block_index) const = 0;
  virtual size_t getMemorySize() const = 0;
  virtual size_t voxels_per_side() const = 0;
  virtual FloatingPoint voxel_size() const = 0;
  virtual FloatingPoint block_size() const = 0;
  virtual std::unique_ptr<ScoreLayer> clone() const = 0;

  // Serialization
  virtual bool saveBlockToStream(BlockIndex block_index,
                                 std::fstream* outfile_ptr) const = 0;
  virtual bool saveBlocksToStream(bool include_all_blocks,
                                  voxblox::BlockIndexList blocks_to_include,
                                  std::fstream* outfile_ptr) const = 0;
  virtual bool addBlockFromProto(const voxblox::BlockProto& block_proto) = 0;

  /**
   * @brief Directly returns the voxel if it exists. Returns nullpointer
   * otherwise. Prefer this when looking up sparse voxels since it avoids the
   * ScoreBlock intermediate step.
   */
  virtual ScoreVoxel* getVoxelPtrByCoordinates(const Point& coords) = 0;
  virtual const ScoreVoxel* getVoxelPtrByCoordinates(
      const Point& coords) const = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_SCORES_SCORE_LAYER_H_
