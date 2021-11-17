#include <queue>
#include <random>

#include <gtest/gtest.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/tests/test_utils.h"
#include "panoptic_mapping/tools/serialization.h"

namespace panoptic_mapping {

namespace test {

// TEST(ClassVoxel, serializeEmptyBlock) {
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
//           kVoxelsPerSide, kVoxelSize, origin);
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
//           kVoxelsPerSide, kVoxelSize, origin);
//   std::vector<uint32_t> data;
//   voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_save, &data, top_n_to_serialize);
//   voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_load, data);
//   checkBlockEqual<panoptic_mapping::ClassVoxel>(&block_to_save,
//                                                   &block_to_load);
// }

TEST(BinaryCountsVoxel, RandomBlock) {
  // Setup a layer with a random block.
  BinaryCountLayer layer(BinaryCountLayer::Config(), kVoxelSize,
                         kVoxelsPerSide);
  auto block = layer.getLayer().allocateBlockPtrByCoordinates(kOrigin);
  std::random_device
      rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
  for (size_t i = 0; i < kVoxelsPerSide * kVoxelsPerSide * kVoxelsPerSide;
       ++i) {
    randomizeVoxel(&block->getVoxelByLinearIndex(i), gen);
  }

  // save the block.
}

// TEST(ClassVoxel, serializeNonGtVoxels) {
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
//           kVoxelsPerSide, kVoxelSize, origin);
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
//           kVoxelsPerSide, kVoxelSize, origin);
//   // Populate block

//   for (int i = 0; i < block_to_save.num_voxels(); i++) {
//     if (i % 100 != 99) {
//       // Leave some voxels uninitialized
//       auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//           block_to_save.computeCoordinatesFromLinearIndex(i));
//       initializeVoxel(voxel, kNumClasses);
//       // Set class
//       panoptic_mapping::classVoxelIncrementClass(voxel, i % kNumClasses);
//     }
//   }
//   std::vector<uint32_t> data;
//   voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_save, &data, top_n_to_serialize);
//   voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_load, data);
//   checkBlockEqual<panoptic_mapping::ClassVoxel>(&block_to_save,
//                                                   &block_to_load);
// }

// TEST(ClassVoxel, serializeGtVoxels) {
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
//           kVoxelsPerSide, kVoxelSize, origin);
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
//           kVoxelsPerSide, kVoxelSize, origin);
//   // Populate block

//   for (int i = 0; i < block_to_save.num_voxels(); i++) {
//     if (i % 100 != 99) {
//       // Leave some voxels uninitialized
//       auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//           block_to_save.computeCoordinatesFromLinearIndex(i));
//       initializeVoxel(voxel, kNumClasses);
//       voxel->is_groundtruth = true;
//       // Set class
//       panoptic_mapping::classVoxelIncrementClass(voxel, i % kNumClasses);
//     }
//   }
//   std::vector<uint32_t> data;
//   voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_save, &data, top_n_to_serialize);
//   voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_load, data);
//   checkBlockEqual<panoptic_mapping::ClassVoxel>(&block_to_save,
//                                                   &block_to_load);
// }

// TEST(ClassVoxel, serializeMultipleCounts) {
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
//           kVoxelsPerSide, kVoxelSize, origin);
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
//           kVoxelsPerSide, kVoxelSize, origin);
//   // Populate block

//   for (int i = 0; i < block_to_save.num_voxels(); i++) {
//     if (i % 100 != 99) {
//       // Leave some voxels uninitialized
//       auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//           block_to_save.computeCoordinatesFromLinearIndex(i));
//       initializeVoxel(voxel, kNumClasses);
//       // Set class
//       panoptic_mapping::classVoxelIncrementClass(voxel, 0);
//       panoptic_mapping::classVoxelIncrementClass(voxel, 1);
//       panoptic_mapping::classVoxelIncrementClass(voxel, 1);
//       panoptic_mapping::classVoxelIncrementClass(voxel, i % kNumClasses);
//       panoptic_mapping::classVoxelIncrementClass(voxel, i % kNumClasses);
//       panoptic_mapping::classVoxelIncrementClass(voxel, i % kNumClasses);
//       panoptic_mapping::classVoxelIncrementClass(voxel, i % kNumClasses);
//     }
//   }
//   std::vector<uint32_t> data;
//   voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_save, &data, top_n_to_serialize);
//   voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_load, data);
//   checkBlockEqual<panoptic_mapping::ClassVoxel>(&block_to_save,
//                                                   &block_to_load);
// }

// TEST(ClassVoxel, moreThanTopNCounts) {
//   /**
//    * This tries to assign more counts than that are serialized (defined in
//    * top_n_to_serialize) E.g. if
//    * top_n_to_serialize = 3, only the top 3 counts are
//    * serialized (to save memory)
//    */
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
//           kVoxelsPerSide, kVoxelSize, origin);
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
//           kVoxelsPerSide, kVoxelSize, origin);
//   // Populate block

//   for (int i = 0; i < block_to_save.num_voxels(); i++) {
//     if (i % 100 != 99) {
//       // Leave some voxels uninitialized
//       auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//           block_to_save.computeCoordinatesFromLinearIndex(i));
//       initializeVoxel(voxel, kNumClasses);

//       EXPECT_LT(top_n_to_serialize, kNumClasses);
//       for (int j = 0; j < top_n_to_serialize + 2; j++) {
//         for (int k = 0; k <= j + 1; k++) {
//           panoptic_mapping::classVoxelIncrementClass(voxel, j);
//         }
//       }
//     }
//   }
//   std::vector<uint32_t> data;
//   voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_save, &data, top_n_to_serialize);
//   voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_load, data);
//   checkBlockEqual<panoptic_mapping::ClassVoxel>(&block_to_save,
//                                                   &block_to_load);
// }

// TEST(ClassUncertaintyVoxel, withUncertainty) {
//   voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_save(
//           kVoxelsPerSide, kVoxelSize, origin);
//   voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_load(
//           kVoxelsPerSide, kVoxelSize, origin);
//   // Populate block

//   for (int i = 0; i < block_to_save.num_voxels(); i++) {
//     if (i % 100 != 99) {
//       auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//           block_to_save.computeCoordinatesFromLinearIndex(i));
//       initializeVoxel(voxel, kNumClasses);

//       EXPECT_LT(top_n_to_serialize, kNumClasses);
//       for (int j = 0; j < top_n_to_serialize + 2; j++) {
//         for (int k = 0; k <= j + 1; k++) {
//           panoptic_mapping::classVoxelIncrementClass(voxel, j);
//         }
//       }
//       panoptic_mapping::classVoxelUpdateUncertainty(voxel, 1.0f / (i + 1));
//     }
//   }
//   std::vector<uint32_t> data;
//   voxblox::serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
//       &block_to_save, &data, top_n_to_serialize);
//   voxblox::deserializeBlockFromIntegers<
//       panoptic_mapping::ClassUncertaintyVoxel>(&block_to_load, data);
//   checkBlockEqual<panoptic_mapping::ClassUncertaintyVoxel>(&block_to_save,
//                                                              &block_to_load);
// }

// /**
//  * Assign random values to voxels
//  */
// TEST(ClassUncertaintyVoxel, randomTesting) {
//   std::random_device
//       rd;  // Will be used to obtain a seed for the random number engine
//   std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with
//   rd() std::uniform_real_distribution<> dis(0.0, 1.0);

//   for (int runs = 0; runs < 10; runs++) {
//     voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_save(
//             kVoxelsPerSide, kVoxelSize, origin);
//     voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_load(
//             kVoxelsPerSide, kVoxelSize, origin);

//     for (int i = 0; i < block_to_save.num_voxels(); i++) {
//       bool skip_voxel = dis(gen) > 0.8;  // Leave 20% uninitialized

//       if (!skip_voxel) {
//         bool use_binary = dis(gen) > 0.8;  // Use binary 20% of the time
//         auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//             block_to_save.computeCoordinatesFromLinearIndex(i));

//         if (use_binary) {
//           int num_votes = static_cast<int>(dis(gen) * 100) + 1;
//           for (int vote = 0; vote < num_votes; vote++) {
//             panoptic_mapping::classVoxelIncrementBinary(voxel, dis(gen) >
//             0.5);
//           }
//         } else {
//           initializeVoxel(voxel, kNumClasses);
//           panoptic_mapping::classVoxelIncrementClass(voxel, 0);

//           // Make sure we have unique top 3 counts
//           std::vector<int> votes;
//           for (int i = 1; i <= kNumClasses; i++) {
//             votes.push_back(i);
//           }
//           std::shuffle(votes.begin(), votes.end(), gen);

//           for (int j = 0; j < votes.size(); j++) {
//             // Leave out some votes randomly
//             if (dis(gen) > 0.1) {
//               for (int k = 0; k <= votes.at(j); k++) {
//                 panoptic_mapping::classVoxelIncrementClass(voxel, j);
//               }
//             }
//           }
//         }
//         // Randomly assign GT
//         voxel->is_groundtruth = dis(gen) > 0.9;
//         // Randomly assign uncertainty
//         panoptic_mapping::classVoxelUpdateUncertainty(voxel, dis(gen));
//       }
//     }
//     std::vector<uint32_t> data;
//     voxblox::serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
//         &block_to_save, &data, top_n_to_serialize);
//     voxblox::deserializeBlockFromIntegers<
//         panoptic_mapping::ClassUncertaintyVoxel>(&block_to_load, data);
//     checkBlockEqual<panoptic_mapping::ClassUncertaintyVoxel>(&block_to_save,
//                                                                &block_to_load);
//   }
// }

// TEST(ClassUncertaintyVoxel, randomTestingDifferentCount) {
//   std::random_device
//           rd;  // Will be used to obtain a seed for the random number engine
//   std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with
//   rd() std::uniform_real_distribution<> dis(0.0, 1.0);

//   for (int runs = 0; runs < 10; runs++) {
//     voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_save(
//             kVoxelsPerSide, kVoxelSize, origin);
//     voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_load(
//             kVoxelsPerSide, kVoxelSize, origin);

//     for (int i = 0; i < block_to_save.num_voxels(); i++) {
//       bool skip_voxel = dis(gen) > 0.8;  // Leave 20% uninitialized

//       if (!skip_voxel) {
//         bool use_binary = dis(gen) > 0.8;  // Use binary 20% of the time
//         auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//                 block_to_save.computeCoordinatesFromLinearIndex(i));

//         if (use_binary) {
//           int num_votes = static_cast<int>(dis(gen) * 100) + 1;
//           for (int vote = 0; vote < num_votes; vote++) {
//             panoptic_mapping::classVoxelIncrementBinary(voxel, dis(gen) >
//             0.5);
//           }
//         } else {
//           initializeVoxel(voxel, kNumClasses);
//           panoptic_mapping::classVoxelIncrementClass(voxel, 0);

//           // Make sure we have unique top 3 counts
//           std::vector<int> votes;
//           for (int i = 1; i <= kNumClasses; i++) {
//             votes.push_back(i);
//           }
//           std::shuffle(votes.begin(), votes.end(), gen);

//           for (int j = 0; j < votes.size(); j++) {
//             // Leave out some votes randomly
//             if (dis(gen) > 0.1) {
//               for (int k = 0; k <= votes.at(j); k++) {
//                 panoptic_mapping::classVoxelIncrementClass(voxel, j);
//               }
//             }
//           }
//         }
//         // Randomly assign GT
//         voxel->is_groundtruth = dis(gen) > 0.9;
//         // Randomly assign uncertainty
//         panoptic_mapping::classVoxelUpdateUncertainty(voxel, dis(gen));
//       }
//     }
//     int old_top_n = top_n_to_serialize;
//     top_n_to_serialize = 4;
//     std::vector<uint32_t> data;
//     voxblox::serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
//             &block_to_save, &data, top_n_to_serialize);
//     voxblox::deserializeBlockFromIntegers<
//             panoptic_mapping::ClassUncertaintyVoxel>(&block_to_load, data);
//     checkBlockEqual<panoptic_mapping::ClassUncertaintyVoxel>(&block_to_save,
//                                                                &block_to_load);
//     top_n_to_serialize = old_top_n;
//   }
// }

/**
 * BINARY TESTS
 */

// TEST(BinaryClassVoxel, binaryClassification) {
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_save(
//           kVoxelsPerSide, kVoxelSize, origin);
//   voxblox::Block<panoptic_mapping::ClassVoxel> block_to_load(
//           kVoxelsPerSide, kVoxelSize, origin);

//   for (int i = 0; i < block_to_save.num_voxels(); i++) {
//     // Leave some voxels uninitialized
//     if (i % 100 != 99) {
//       auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//           block_to_save.computeCoordinatesFromLinearIndex(i));
//       panoptic_mapping::classVoxelIncrementBinary(voxel, i % 2);
//     }
//   }
//   std::vector<uint32_t> data;
//   voxblox::serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_save, &data, top_n_to_serialize);
//   voxblox::deserializeBlockFromIntegers<panoptic_mapping::ClassVoxel>(
//       &block_to_load, data);
//   checkBlockEqual<panoptic_mapping::ClassVoxel>(&block_to_save,
//                                                   &block_to_load);
// }

// TEST(BinaryClassUncertaintyVoxel, binaryClassification) {
//   voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_save(
//           kVoxelsPerSide, kVoxelSize, origin);
//   voxblox::Block<panoptic_mapping::ClassUncertaintyVoxel> block_to_load(
//           kVoxelsPerSide, kVoxelSize, origin);

//   for (int i = 0; i < block_to_save.num_voxels(); i++) {
//     // Leave some voxels uninitialized
//     if (i % 100 != 99) {
//       auto* voxel = block_to_save.getVoxelPtrByCoordinates(
//           block_to_save.computeCoordinatesFromLinearIndex(i));
//       panoptic_mapping::classVoxelIncrementBinary(voxel, i % 2);
//       panoptic_mapping::classVoxelUpdateUncertainty(voxel, 1.0 / (i + 1));
//     }
//   }
//   std::vector<uint32_t> data;
//   voxblox::serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
//       &block_to_save, &data, top_n_to_serialize);
//   voxblox::deserializeBlockFromIntegers<
//       panoptic_mapping::ClassUncertaintyVoxel>(&block_to_load, data);
//   checkBlockEqual<panoptic_mapping::ClassUncertaintyVoxel>(&block_to_save,
//                                                              &block_to_load);
// }

}  // namespace test

}  // namespace panoptic_mapping

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}