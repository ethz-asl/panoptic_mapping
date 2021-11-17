#ifndef PANOPTIC_MAPPING_TEST_UTILS_H_
#define PANOPTIC_MAPPING_TEST_UTILS_H_
#include <random>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/binary_counts.h"

namespace panoptic_mapping {
namespace test {

// Test Configuration.
const int kVoxelsPerSide = 16;
const int kNumClasses = 40;
const float kVoxelSize = 0.1;
const voxblox::Point kOrigin(0, 0, 0);
const int k_num_classes = 40;
int top_n_to_serialize = 12;
const float kTol = 0.00001;

// Assertions.
inline void checkVoxelEqual(const panoptic_mapping::TsdfVoxel& v1,
                            const panoptic_mapping::TsdfVoxel& v2) {
  EXPECT_NEAR(v1.distance, v2.distance, kTol);
  EXPECT_NEAR(v1.weight, v2.weight, kTol);
  EXPECT_EQ(v1.color.r, v2.color.r);
  EXPECT_EQ(v1.color.g, v2.color.g);
  EXPECT_EQ(v1.color.b, v2.color.b);
}

inline void checkVoxelEqual(const panoptic_mapping::BinaryCountVoxel& v1,
                            const panoptic_mapping::BinaryCountVoxel& v2) {
  EXPECT_EQ(v1.belongs_count, v2.belongs_count);
  EXPECT_EQ(v1.foreign_count, v2.foreign_count);
}

template <typename T>
inline void checkBlockEqual(const voxblox::Block<T>& blk1,
                            const voxblox::Block<T>& blk2) {
  for (int i = 0; i < blk1.num_voxels(); i++) {
    checkVoxelEqual(blk1.getVoxelByLinearIndex(i),
                    blk2.getVoxelByLinearIndex(i));
  }
}

template <typename T>
inline void checkLayerEqual(const voxblox::Layer<T>& layer1,
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
  for (const BlockIndex& index : indices) {
    checkBlockEqual(layer1.getBlockByIndex(index),
                    layer2.getBlockByIndex(index));
  }
}

// Random Initialization.
void randomizeVoxel(TsdfVoxel* voxel) {
  std::random_device
      rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(0.0, 1.0);

  // TSDF Voxel
  voxel->distance = static_cast<float>(dis(gen));
  voxel->weight = static_cast<float>(dis(gen));
  voxel->color = panoptic_mapping::Color(static_cast<uint8_t>(dis(gen) * 255),
                                         static_cast<uint8_t>(dis(gen) * 255),
                                         static_cast<uint8_t>(dis(gen) * 255));

  //   if (!class_voxel) {
  //     return;
  //   }
  //   // Class Voxel
  //   bool use_binary = dis(gen) > 0.8;  // Use binary 20% of the time

  //   if (use_binary) {
  //     int num_votes = static_cast<int>(dis(gen) * 100) + 1;
  //     for (int vote = 0; vote < num_votes; vote++) {
  //       classVoxelIncrementBinary(class_voxel, dis(gen) > 0.5);
  //     }
  //   } else {
  //     // Initialize counts
  //     for (int i = 0; i < k_num_classes; ++i)
  //     class_voxel->counts.push_back(0); classVoxelIncrementClass(class_voxel,
  //     0);

  //     // Make sure we have unique top 3 counts
  //     std::vector<int> votes;
  //     for (int i = 1; i <= k_num_classes; i++) {
  //       votes.push_back(i);
  //     }
  //     std::shuffle(votes.begin(), votes.end(), gen);

  //     for (int j = 0; j < votes.size(); j++) {
  //       // Leave out some votes randomly
  //       if (dis(gen) > 0.1) {
  //         for (int k = 0; k <= votes.at(j); k++) {
  //           classVoxelIncrementClass(class_voxel, j);
  //         }
  //       }
  //     }
  //   }
  //   // Randomly assign GT
  //   class_voxel->is_groundtruth = dis(gen) > 0.9;
  //   // Randomly assign uncertainty
  //   classVoxelUpdateUncertainty(class_voxel, dis(gen));
}

void randomizeVoxel(BinaryCountVoxel* voxel, const std::mt19937& random_gen) {
  std::uniform_int_distribution<unsigned short> distrib();
  voxel->belongs_count = distrib(random_gen);
  voxel->belongs_count = distrib(random_gen);
}

}  // namespace test
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TEST_UTILS_H_
