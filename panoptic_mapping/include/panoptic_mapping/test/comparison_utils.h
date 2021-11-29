#ifndef PANOPTIC_MAPPING_TEST_COMPARISON_UTILS_H_
#define PANOPTIC_MAPPING_TEST_COMPARISON_UTILS_H_
#include <random>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/binary_count.h"

namespace panoptic_mapping {
namespace test {

// Test Configuration.
const int kVoxelsPerSide = 16;
const int kNumClasses = 40;
const float kVoxelSize = 0.1;
const Point kOrigin(0, 0, 0);
const int k_num_classes = 40;
int top_n_to_serialize = 12;
const float kTol = 0.00001;

// Assertions.
inline bool checkVoxelEqual(const panoptic_mapping::TsdfVoxel& v1,
                            const panoptic_mapping::TsdfVoxel& v2) {
  EXPECT_NEAR(v1.distance, v2.distance, kTol);
  EXPECT_NEAR(v1.weight, v2.weight, kTol);
  EXPECT_EQ(v1.color.r, v2.color.r);
  EXPECT_EQ(v1.color.g, v2.color.g);
  EXPECT_EQ(v1.color.b, v2.color.b);
  return (v1.color.r == v2.color.r && v1.color.g == v2.color.g &&
          v1.color.b == v2.color.b &&
          std::fabs(v1.distance - v2.distance) <= kTol &&
          std::fabs(v1.weight - v2.weight) <= kTol);
}

inline bool checkVoxelEqual(const panoptic_mapping::BinaryCountVoxel& v1,
                            const panoptic_mapping::BinaryCountVoxel& v2) {
  EXPECT_EQ(v1.belongs_count, v2.belongs_count);
  EXPECT_EQ(v1.foreign_count, v2.foreign_count);
  return v1.belongs_count == v2.belongs_count &&
         v1.foreign_count == v2.foreign_count;
}

template <typename T>
inline bool checkBlockEqual(const voxblox::Block<T>& blk1,
                            const voxblox::Block<T>& blk2) {
  bool all_voxels_equal = true;
  for (int i = 0; i < blk1.num_voxels(); i++) {
    all_voxels_equal =
        all_voxels_equal && checkVoxelEqual(blk1.getVoxelByLinearIndex(i),
                                            blk2.getVoxelByLinearIndex(i));
  }
  EXPECT_TRUE(all_voxels_equal);

  return all_voxels_equal;
}

template <typename T>
inline bool checkLayerEqual(const voxblox::Layer<T>& layer1,
                            const voxblox::Layer<T>& layer2) {
  // Check number of blocks.
  voxblox::BlockIndexList indices;
  EXPECT_EQ(layer1.getNumberOfAllocatedBlocks(),
            layer2.getNumberOfAllocatedBlocks());
  layer1.getAllAllocatedBlocks(&indices);
  for (const BlockIndex& index : indices) {
    EXPECT_TRUE(layer2.hasBlock(index));
  }

  // Check content of blocks
  bool all_blocks_equal = true;
  for (const BlockIndex& index : indices) {
    all_blocks_equal =
        all_blocks_equal && checkBlockEqual(layer1.getBlockByIndex(index),
                                            layer2.getBlockByIndex(index));
  }
  EXPECT_TRUE(all_blocks_equal);
  return all_blocks_equal;
}

}  // namespace test
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TEST_COMPARISON_UTILS_H_
