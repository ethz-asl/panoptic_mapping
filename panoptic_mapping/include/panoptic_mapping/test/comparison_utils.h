#ifndef PANOPTIC_MAPPING_TEST_COMPARISON_UTILS_H_
#define PANOPTIC_MAPPING_TEST_COMPARISON_UTILS_H_

#include <random>
#include <string>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/binary_count.h"
#include "panoptic_mapping/map/classification/fixed_count.h"
#include "panoptic_mapping/map/classification/moving_binary_count.h"
#include "panoptic_mapping/map/classification/uncertainty.h"
#include "panoptic_mapping/map/classification/variable_count.h"

namespace panoptic_mapping {
namespace test {

// Test Configuration.
const float kTol = 0.00001;

// Assertions.
inline bool checkVoxelEqual(const TsdfVoxel& v1, const TsdfVoxel& v2) {
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

inline bool checkVoxelEqual(const BinaryCountVoxel& v1,
                            const BinaryCountVoxel& v2) {
  EXPECT_EQ(v1.belongs_count, v2.belongs_count);
  EXPECT_EQ(v1.foreign_count, v2.foreign_count);
  return v1.belongs_count == v2.belongs_count &&
         v1.foreign_count == v2.foreign_count;
}

inline bool checkVoxelEqual(const FixedCountVoxel& v1,
                            const FixedCountVoxel& v2) {
  EXPECT_EQ(v1.counts.size(), v2.counts.size());
  EXPECT_EQ(v1.current_index, v2.current_index);
  EXPECT_EQ(v1.current_count, v2.current_count);
  EXPECT_EQ(v1.total_count, v2.total_count);
  EXPECT_EQ(v1.counts, v2.counts);

  return v1.counts.size() == v2.counts.size() &&
         v1.current_index == v2.current_index &&
         v1.current_count == v2.current_count &&
         v1.total_count == v2.total_count && v1.counts == v2.counts;
}

inline bool checkVoxelEqual(const MovingBinaryCountVoxel& v1,
                            const MovingBinaryCountVoxel& v2) {
  EXPECT_EQ(v1.belongs_count, v2.belongs_count);
  EXPECT_EQ(v1.foreign_count, v2.foreign_count);
  return v1.belongs_count == v2.belongs_count &&
         v1.foreign_count == v2.foreign_count;
}

inline bool checkVoxelEqual(const VariableCountVoxel& v1,
                            const VariableCountVoxel& v2) {
  EXPECT_EQ(v1.counts.size(), v2.counts.size());
  // NOTE(schmluk): We don't check the current index since this can be ambiguous
  // for identical maximum counts.
  EXPECT_EQ(v1.current_count, v2.current_count);
  EXPECT_EQ(v1.total_count, v2.total_count);
  for (const auto& id_count_pair : v1.counts) {
    auto it = v2.counts.find(id_count_pair.first);
    if (it == v2.counts.end()) {
      // This is an ugly work around since FAIL() apparently returns void...
      const std::string error = "ID " + std::to_string(id_count_pair.first) +
                                " of v1 not found in v2.";
      EXPECT_EQ(error, "");
      return false;
    }
    EXPECT_EQ(id_count_pair.first, it->first);
    EXPECT_EQ(id_count_pair.second, it->second);
    if (id_count_pair.first != it->first ||
        id_count_pair.second != it->second) {
      return false;
    }
  }
  return v1.counts.size() == v2.counts.size() &&
         v1.current_count == v2.current_count &&
         v1.total_count == v2.total_count;
}

inline bool checkVoxelEqual(const UncertaintyVoxel& v1,
                            const UncertaintyVoxel& v2) {
  if (!checkVoxelEqual(static_cast<const FixedCountVoxel&>(v1),
                       static_cast<const FixedCountVoxel&>(v2))) {
    return false;
  }
  EXPECT_EQ(v1.is_ground_truth, v2.is_ground_truth);
  EXPECT_EQ(v1.uncertainty, v2.uncertainty);
  return v1.is_ground_truth == v2.is_ground_truth &&
         v1.uncertainty == v2.uncertainty;
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
