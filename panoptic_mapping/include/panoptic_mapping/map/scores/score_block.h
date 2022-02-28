#ifndef PANOPTIC_MAPPING_MAP_SCORES_SCORE_BLOCK_H_
#define PANOPTIC_MAPPING_MAP_SCORES_SCORE_BLOCK_H_

#include <memory>
#include <utility>

#include <voxblox/core/block.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/scores/score_voxel.h"

namespace panoptic_mapping {

/**
 * @brief General interface for score voxel blocks. Wraps the voxblox
 * block to allow substituting different classification layer types.
 */
class ScoreBlock {
 public:
  // Wrap the shared pointer operator bool to account for the wrapped blocks.
  class Ptr : public std::shared_ptr<ScoreBlock> {
   public:
    explicit Ptr(ScoreBlock* ptr = nullptr)
        : std::shared_ptr<ScoreBlock>(ptr) {}
    operator bool() const {
      if (!std::shared_ptr<ScoreBlock>::operator bool()) {
        return false;
      }
      return this->operator->()->isValid();
    }
  };
  class ConstPtr : public std::shared_ptr<const ScoreBlock> {
   public:
    explicit ConstPtr(const ScoreBlock* ptr = nullptr)
        : std::shared_ptr<const ScoreBlock>(ptr) {}
    operator bool() const {
      if (!std::shared_ptr<const ScoreBlock>::operator bool()) {
        return false;
      }
      return this->operator->()->isValid();
    }
  };

  virtual ~ScoreBlock() = default;

  // Implements all voxel access interfaces.
  virtual const ScoreVoxel& getVoxelByLinearIndex(size_t index) const = 0;
  virtual const ScoreVoxel& getVoxelByVoxelIndex(
      const VoxelIndex& index) const = 0;
  virtual const ScoreVoxel& getVoxelByCoordinates(
      const Point& coords) const = 0;
  virtual ScoreVoxel& getVoxelByCoordinates(const Point& coords) = 0;
  virtual ScoreVoxel* getVoxelPtrByCoordinates(const Point& coords) = 0;
  virtual const ScoreVoxel* getVoxelPtrByCoordinates(
      const Point& coords) const = 0;
  virtual ScoreVoxel& getVoxelByLinearIndex(size_t index) = 0;
  virtual ScoreVoxel& getVoxelByVoxelIndex(const VoxelIndex& index) = 0;
  virtual ScoreVoxelType getVoxelType() const = 0;

  // Additional checks for validity.
  operator bool() const { return isValid(); }

 protected:
  virtual bool isValid() const = 0;
};

/**
 * @brief Implements all the interface fucntions to wrap a block of VoxelT
 * voxels.
 *
 * @tparam VoxelT Voxel type contained in the wrapped block. Needs to inherit
 * from ScoreVoxel.
 */
template <typename VoxelT>
class ScoreBlockImpl : public ScoreBlock {
 public:
  explicit ScoreBlockImpl(
      std::shared_ptr<voxblox::Block<VoxelT>> block = nullptr)
      : block_(std::move(block)) {}

  const ScoreVoxel& getVoxelByLinearIndex(size_t index) const override {
    return block_->getVoxelByLinearIndex(index);
  }
  const ScoreVoxel& getVoxelByVoxelIndex(
      const VoxelIndex& index) const override {
    return block_->getVoxelByVoxelIndex(index);
  }
  const ScoreVoxel& getVoxelByCoordinates(const Point& coords) const override {
    return block_->getVoxelByCoordinates(coords);
  }
  ScoreVoxel& getVoxelByCoordinates(const Point& coords) override {
    return block_->getVoxelByCoordinates(coords);
  }
  ScoreVoxel* getVoxelPtrByCoordinates(const Point& coords) override {
    return block_->getVoxelPtrByCoordinates(coords);
  }
  const ScoreVoxel* getVoxelPtrByCoordinates(
      const Point& coords) const override {
    return block_->getVoxelPtrByCoordinates(coords);
  }
  ScoreVoxel& getVoxelByLinearIndex(size_t index) override {
    return block_->getVoxelByLinearIndex(index);
  }
  ScoreVoxel& getVoxelByVoxelIndex(const VoxelIndex& index) override {
    return block_->getVoxelByVoxelIndex(index);
  }
  ScoreVoxelType getVoxelType() const override {
    return reinterpret_cast<const ScoreVoxel&>(block_->getVoxelByLinearIndex(0))
        .getVoxelType();
  }

  // Exposes the actual block if the type is known.
  voxblox::Block<VoxelT>& getBlock() { return *block_; }
  const voxblox::Block<VoxelT>& getBlock() const { return *block_; }

 protected:
  const std::shared_ptr<voxblox::Block<VoxelT>> block_;
  bool isValid() const override { return block_.operator bool(); }
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_SCORES_SCORE_BLOCK_H_
