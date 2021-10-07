#include <queue>
#include <random>

#include <gtest/gtest.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/tools/serialization.h"

const int voxels_per_side = 16;
const int num_classes = 40;
const float voxel_size = 0.1;
const voxblox::Point origin(0, 0, 0);
int top_n_to_serialize = 12;

/** HELPER FUNCTIONS **/
inline void initialize_voxel(panoptic_mapping::ClassVoxel* voxel,
                             int num_classes) {
  for (int i = 0; i < num_classes; i++) {
    voxel->counts.push_back(0);
  }
}

inline void check_voxel_equal(const panoptic_mapping::ClassVoxel& v1,
                              const panoptic_mapping::ClassVoxel& v2) {
  EXPECT_EQ(v1.is_gt, v2.is_gt);
  EXPECT_EQ(v1.current_index, v2.current_index);
  EXPECT_EQ(v1.belongs_count, v2.belongs_count);
  EXPECT_EQ(v1.foreign_count, v2.foreign_count);
  EXPECT_EQ(v1.counts.size(), v2.counts.size());
  if (v1.counts.empty()) {
    return;
  }
  // Only check if top_n counts match:
  std::priority_queue<std::pair<int, int>> class_idx_to_count_1;
  for (int i = 0; i < v1.counts.size(); ++i) {
    class_idx_to_count_1.push(std::pair<int, int>(v1.counts.at(i), i));
  }
  std::priority_queue<std::pair<int, int>> class_idx_to_count_2;
  for (int i = 0; i < v2.counts.size(); ++i) {
    class_idx_to_count_2.push(std::pair<int, int>(v2.counts.at(i), i));
  }
  for (int i = 0; i < top_n_to_serialize; ++i) {
    EXPECT_EQ(class_idx_to_count_1.top().first,
              class_idx_to_count_2.top().first);
    EXPECT_EQ(class_idx_to_count_1.top().second,
              class_idx_to_count_2.top().second);
    class_idx_to_count_1.pop();
    class_idx_to_count_2.pop();
  }
}

inline void check_voxel_equal(
    const panoptic_mapping::ClassUncertaintyVoxel& v1,
    const panoptic_mapping::ClassUncertaintyVoxel& v2) {
  check_voxel_equal(static_cast<panoptic_mapping::ClassVoxel>(v1),
                    static_cast<panoptic_mapping::ClassVoxel>(v2));
  EXPECT_EQ(v1.uncertainty_value, v2.uncertainty_value);
}

template <typename T>
inline void check_block_equal(const voxblox::Block<T>* blk1,
                              const voxblox::Block<T>* blk2) {
  for (int i = 0; i < blk1->num_voxels(); i++) {
    check_voxel_equal(blk1->getVoxelByLinearIndex(i),
                      blk2->getVoxelByLinearIndex(i));
  }
}

/** START TESTS **/

TEST(ClassVoxel, serializeEmptyBlock) {
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
      voxels_per_side, voxel_size, origin);
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
      voxels_per_side, voxel_size, origin);
  std::vector<uint32_t> data;
  voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_save, &data, top_n_to_serialize);
  voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_load, data);
  check_block_equal<panoptic_mapping::ClassVoxel>(&block_to_save,
                                                  &block_to_load);
}

TEST(ClassVoxel, serializeNonGtVoxels) {
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
      voxels_per_side, voxel_size, origin);
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
      voxels_per_side, voxel_size, origin);
  // Populate block

  for (int i = 0; i < block_to_save.num_voxels(); i++) {
    if (i % 100 != 99) {
      // Leave some voxels uninitialized
      auto* voxel = block_to_save.getVoxelPtrByCoordinates(
          block_to_save.computeCoordinatesFromLinearIndex(i));
      initialize_voxel(voxel, num_classes);
      // Set class
      panoptic_mapping::classVoxelIncrementClass(voxel, i % num_classes);
    }
  }
  std::vector<uint32_t> data;
  voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_save, &data, top_n_to_serialize);
  voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_load, data);
  check_block_equal<panoptic_mapping::ClassVoxel>(&block_to_save,
                                                  &block_to_load);
}

TEST(ClassVoxel, serializeGtVoxels) {
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
      voxels_per_side, voxel_size, origin);
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
      voxels_per_side, voxel_size, origin);
  // Populate block

  for (int i = 0; i < block_to_save.num_voxels(); i++) {
    if (i % 100 != 99) {
      // Leave some voxels uninitialized
      auto* voxel = block_to_save.getVoxelPtrByCoordinates(
          block_to_save.computeCoordinatesFromLinearIndex(i));
      initialize_voxel(voxel, num_classes);
      voxel->is_gt = true;
      // Set class
      panoptic_mapping::classVoxelIncrementClass(voxel, i % num_classes);
    }
  }
  std::vector<uint32_t> data;
  voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_save, &data, top_n_to_serialize);
  voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_load, data);
  check_block_equal<panoptic_mapping::ClassVoxel>(&block_to_save,
                                                  &block_to_load);
}

TEST(ClassVoxel, serializeMultipleCounts) {
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
      voxels_per_side, voxel_size, origin);
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
      voxels_per_side, voxel_size, origin);
  // Populate block

  for (int i = 0; i < block_to_save.num_voxels(); i++) {
    if (i % 100 != 99) {
      // Leave some voxels uninitialized
      auto* voxel = block_to_save.getVoxelPtrByCoordinates(
          block_to_save.computeCoordinatesFromLinearIndex(i));
      initialize_voxel(voxel, num_classes);
      // Set class
      panoptic_mapping::classVoxelIncrementClass(voxel, 0);
      panoptic_mapping::classVoxelIncrementClass(voxel, 1);
      panoptic_mapping::classVoxelIncrementClass(voxel, 1);
      panoptic_mapping::classVoxelIncrementClass(voxel, i % num_classes);
      panoptic_mapping::classVoxelIncrementClass(voxel, i % num_classes);
      panoptic_mapping::classVoxelIncrementClass(voxel, i % num_classes);
      panoptic_mapping::classVoxelIncrementClass(voxel, i % num_classes);
    }
  }
  std::vector<uint32_t> data;
  voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_save, &data, top_n_to_serialize);
  voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_load, data);
  check_block_equal<panoptic_mapping::ClassVoxel>(&block_to_save,
                                                  &block_to_load);
}

TEST(ClassVoxel, moreThanTopNCounts) {
  /**
   * This tries to assign more counts than that are serialized (defined in
   * top_n_to_serialize) E.g. if
   * top_n_to_serialize = 3, only the top 3 counts are
   * serialized (to save memory)
   */
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
      voxels_per_side, voxel_size, origin);
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
      voxels_per_side, voxel_size, origin);
  // Populate block

  for (int i = 0; i < block_to_save.num_voxels(); i++) {
    if (i % 100 != 99) {
      // Leave some voxels uninitialized
      auto* voxel = block_to_save.getVoxelPtrByCoordinates(
          block_to_save.computeCoordinatesFromLinearIndex(i));
      initialize_voxel(voxel, num_classes);

      EXPECT_LT(top_n_to_serialize, num_classes);
      for (int j = 0; j < top_n_to_serialize + 2; j++) {
        for (int k = 0; k <= j + 1; k++) {
          panoptic_mapping::classVoxelIncrementClass(voxel, j);
        }
      }
    }
  }
  std::vector<uint32_t> data;
  voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_save, &data, top_n_to_serialize);
  voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_load, data);
  check_block_equal<panoptic_mapping::ClassVoxel>(&block_to_save,
                                                  &block_to_load);
}

TEST(ClassUncertaintyVoxel, withUncertainty) {
  voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_save(
      voxels_per_side, voxel_size, origin);
  voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_load(
      voxels_per_side, voxel_size, origin);
  // Populate block

  for (int i = 0; i < block_to_save.num_voxels(); i++) {
    if (i % 100 != 99) {
      auto* voxel = block_to_save.getVoxelPtrByCoordinates(
          block_to_save.computeCoordinatesFromLinearIndex(i));
      initialize_voxel(voxel, num_classes);

      EXPECT_LT(top_n_to_serialize, num_classes);
      for (int j = 0; j < top_n_to_serialize + 2; j++) {
        for (int k = 0; k <= j + 1; k++) {
          panoptic_mapping::classVoxelIncrementClass(voxel, j);
        }
      }
      panoptic_mapping::classVoxelUpdateUncertainty(voxel, 1.0f / (i + 1));
    }
  }
  std::vector<uint32_t> data;
  voxblox::serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
      &block_to_save, &data, top_n_to_serialize);
  voxblox::deserializeBlockFromIntegers<
      panoptic_mapping::ClassUncertaintyVoxel>(&block_to_load, data);
  check_block_equal<panoptic_mapping::ClassUncertaintyVoxel>(&block_to_save,
                                                             &block_to_load);
}

/**
 * Assign random values to voxels
 */
TEST(ClassUncertaintyVoxel, randomTesting) {
  std::random_device
      rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(0.0, 1.0);

  for (int runs = 0; runs < 10; runs++) {
    voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_save(
        voxels_per_side, voxel_size, origin);
    voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_load(
        voxels_per_side, voxel_size, origin);

    for (int i = 0; i < block_to_save.num_voxels(); i++) {
      bool skip_voxel = dis(gen) > 0.8;  // Leave 20% uninitialized

      if (!skip_voxel) {
        bool use_binary = dis(gen) > 0.8;  // Use binary 20% of the time
        auto* voxel = block_to_save.getVoxelPtrByCoordinates(
            block_to_save.computeCoordinatesFromLinearIndex(i));

        if (use_binary) {
          int num_votes = static_cast<int>(dis(gen) * 100) + 1;
          for (int vote = 0; vote < num_votes; vote++) {
            panoptic_mapping::classVoxelIncrementBinary(voxel, dis(gen) > 0.5);
          }
        } else {
          initialize_voxel(voxel, num_classes);
          panoptic_mapping::classVoxelIncrementClass(voxel, 0);

          // Make sure we have unique top 3 counts
          std::vector<int> votes;
          for (int i = 1; i <= num_classes; i++) {
            votes.push_back(i);
          }
          std::shuffle(votes.begin(), votes.end(), gen);

          for (int j = 0; j < votes.size(); j++) {
            // Leave out some votes randomly
            if (dis(gen) > 0.1) {
              for (int k = 0; k <= votes.at(j); k++) {
                panoptic_mapping::classVoxelIncrementClass(voxel, j);
              }
            }
          }
        }
        // Randomly assign GT
        voxel->is_gt = dis(gen) > 0.9;
        // Randomly assign uncertainty
        panoptic_mapping::classVoxelUpdateUncertainty(voxel, dis(gen));
      }
    }
    std::vector<uint32_t> data;
    voxblox::serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
        &block_to_save, &data, top_n_to_serialize);
    voxblox::deserializeBlockFromIntegers<
        panoptic_mapping::ClassUncertaintyVoxel>(&block_to_load, data);
    check_block_equal<panoptic_mapping::ClassUncertaintyVoxel>(&block_to_save,
                                                               &block_to_load);
  }
}


TEST(ClassUncertaintyVoxel, randomTestingDifferentCount) {
  std::random_device
          rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(0.0, 1.0);

  for (int runs = 0; runs < 10; runs++) {
    voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_save(
            voxels_per_side, voxel_size, origin);
    voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_load(
            voxels_per_side, voxel_size, origin);

    for (int i = 0; i < block_to_save.num_voxels(); i++) {
      bool skip_voxel = dis(gen) > 0.8;  // Leave 20% uninitialized

      if (!skip_voxel) {
        bool use_binary = dis(gen) > 0.8;  // Use binary 20% of the time
        auto* voxel = block_to_save.getVoxelPtrByCoordinates(
                block_to_save.computeCoordinatesFromLinearIndex(i));

        if (use_binary) {
          int num_votes = static_cast<int>(dis(gen) * 100) + 1;
          for (int vote = 0; vote < num_votes; vote++) {
            panoptic_mapping::classVoxelIncrementBinary(voxel, dis(gen) > 0.5);
          }
        } else {
          initialize_voxel(voxel, num_classes);
          panoptic_mapping::classVoxelIncrementClass(voxel, 0);

          // Make sure we have unique top 3 counts
          std::vector<int> votes;
          for (int i = 1; i <= num_classes; i++) {
            votes.push_back(i);
          }
          std::shuffle(votes.begin(), votes.end(), gen);

          for (int j = 0; j < votes.size(); j++) {
            // Leave out some votes randomly
            if (dis(gen) > 0.1) {
              for (int k = 0; k <= votes.at(j); k++) {
                panoptic_mapping::classVoxelIncrementClass(voxel, j);
              }
            }
          }
        }
        // Randomly assign GT
        voxel->is_gt = dis(gen) > 0.9;
        // Randomly assign uncertainty
        panoptic_mapping::classVoxelUpdateUncertainty(voxel, dis(gen));
      }
    }
    int old_top_n = top_n_to_serialize;
    top_n_to_serialize = 4;
    std::vector<uint32_t> data;
    voxblox::serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
            &block_to_save, &data, top_n_to_serialize);
    voxblox::deserializeBlockFromIntegers<
            panoptic_mapping::ClassUncertaintyVoxel>(&block_to_load, data);
    check_block_equal<panoptic_mapping::ClassUncertaintyVoxel>(&block_to_save,
                                                               &block_to_load);
    top_n_to_serialize = old_top_n;
  }
}

/**
 * BINARY TESTS
 */

TEST(BinaryClassVoxel, binaryClassification) {
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
      voxels_per_side, voxel_size, origin);
  voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
      voxels_per_side, voxel_size, origin);

  for (int i = 0; i < block_to_save.num_voxels(); i++) {
    // Leave some voxels uninitialized
    if (i % 100 != 99) {
      auto* voxel = block_to_save.getVoxelPtrByCoordinates(
          block_to_save.computeCoordinatesFromLinearIndex(i));
      panoptic_mapping::classVoxelIncrementBinary(voxel, i % 2);
    }
  }
  std::vector<uint32_t> data;
  voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_save, &data, top_n_to_serialize);
  voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
      &block_to_load, data);
  check_block_equal<panoptic_mapping::ClassVoxel>(&block_to_save,
                                                  &block_to_load);
}

TEST(BinaryClassUncertaintyVoxel, binaryClassification) {
  voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_save(
      voxels_per_side, voxel_size, origin);
  voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_load(
      voxels_per_side, voxel_size, origin);

  for (int i = 0; i < block_to_save.num_voxels(); i++) {
    // Leave some voxels uninitialized
    if (i % 100 != 99) {
      auto* voxel = block_to_save.getVoxelPtrByCoordinates(
          block_to_save.computeCoordinatesFromLinearIndex(i));
      panoptic_mapping::classVoxelIncrementBinary(voxel, i % 2);
      panoptic_mapping::classVoxelUpdateUncertainty(voxel, 1.0 / (i + 1));
    }
  }
  std::vector<uint32_t> data;
  voxblox::serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
      &block_to_save, &data, top_n_to_serialize);
  voxblox::deserializeBlockFromIntegers<
      panoptic_mapping::ClassUncertaintyVoxel>(&block_to_load, data);
  check_block_equal<panoptic_mapping::ClassUncertaintyVoxel>(&block_to_save,
                                                             &block_to_load);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}