#include "queue"
#include <cstdlib>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"

namespace panoptic_mapping {

    const int k_num_blocks = 10;
    const int k_num_classes = 40;
    const int k_num_counts = 5;

    inline void check_voxel_equal(const ClassVoxel v1, const ClassVoxel v2) {
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
      for (int i = 0; i < k_num_counts; ++i) {
        EXPECT_EQ(class_idx_to_count_1.top().first,
                  class_idx_to_count_2.top().first);
        EXPECT_EQ(class_idx_to_count_1.top().second,
                  class_idx_to_count_2.top().second);
        class_idx_to_count_1.pop();
        class_idx_to_count_2.pop();
      }
    }

    inline void check_voxel_equal(const ClassUncertaintyVoxel v1,
                                  const ClassUncertaintyVoxel v2) {
      check_voxel_equal(static_cast<ClassVoxel>(v1), static_cast<ClassVoxel>(v2));
      EXPECT_EQ(v1.uncertainty_value, v2.uncertainty_value);
    }

    inline void check_voxel_equal(const TsdfVoxel v1, const TsdfVoxel v2) {
      EXPECT_EQ(v1.color.r, v2.color.r);
      EXPECT_EQ(v1.color.g, v2.color.g);
      EXPECT_EQ(v1.color.b, v2.color.b);
      EXPECT_NEAR(v1.weight, v2.weight, 0.0001);
      EXPECT_NEAR(v1.distance, v2.distance, 0.0001);
    }

    template<typename T>
    inline void check_block_equal(const voxblox::Block<T> *blk1,
                                  const voxblox::Block<T> *blk2) {
      for (int i = 0; i < blk1->num_voxels(); i++) {
        check_voxel_equal(blk1->getVoxelByLinearIndex(i),
                          blk2->getVoxelByLinearIndex(i));
      }
    }

/**
 * Randomly assigns values to the voxels.
 */
    void load_random_voxels(TsdfVoxel *tsdf_voxel, ClassVoxelType *class_voxel) {
      std::random_device
              rd;  // Will be used to obtain a seed for the random number engine
      std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
      std::uniform_real_distribution<> dis(0.0, 1.0);
      bool skip_voxel = dis(gen) > 0.8;  // Leave 20% uninitialized
      if (skip_voxel) {
        return;
      }

      // TSDF Voxel
      tsdf_voxel->distance = static_cast<float>(dis(gen));
      tsdf_voxel->weight = static_cast<float>(dis(gen));
      tsdf_voxel->color = Color(static_cast<uint8_t>(dis(gen) * 255),
                                static_cast<uint8_t>(dis(gen) * 255),
                                static_cast<uint8_t>(dis(gen) * 255));

      if (!class_voxel) {
        return;
      }
      // Class Voxel
      bool use_binary = dis(gen) > 0.8;  // Use binary 20% of the time

      if (use_binary) {
        int num_votes = static_cast<int>(dis(gen) * 100) + 1;
        for (int vote = 0; vote < num_votes; vote++) {
          classVoxelIncrementBinary(class_voxel, dis(gen) > 0.5);
        }
      } else {
        // Initialize counts
        for (int i = 0; i < k_num_classes; ++i) class_voxel->counts.push_back(0);
        classVoxelIncrementClass(class_voxel, 0);

        // Make sure we have unique top 3 counts
        std::vector<int> votes;
        for (int i = 1; i <= k_num_classes; i++) {
          votes.push_back(i);
        }
        std::shuffle(votes.begin(), votes.end(), gen);

        for (int j = 0; j < votes.size(); j++) {
          // Leave out some votes randomly
          if (dis(gen) > 0.1) {
            for (int k = 0; k <= votes.at(j); k++) {
              classVoxelIncrementClass(class_voxel, j);
            }
          }
        }
      }
      // Randomly assign GT
      class_voxel->is_gt = dis(gen) > 0.9;
      // Randomly assign uncertainty
      classVoxelUpdateUncertainty(class_voxel, dis(gen));
    }

/**
 * Returns a random submap containing class and tsdf voxels.
 * The voxels are calculated randomly thus they will not make any sense. Do not
 * try to visualize this map or extract meshes
 * @return submap ptr
 */
    std::unique_ptr<Submap> getSubmap(bool fill, bool with_class_layer) {
      Submap::Config cfg;
      cfg.use_class_layer = with_class_layer;

      auto submap = std::make_unique<Submap>(cfg);

      // Load the submap data.
      submap->setInstanceID(0);
      submap->setClassID(1);
      submap->setName("test_case");

      if (fill) {
        auto tsdf_layer = submap->getTsdfLayerPtr();
        std::shared_ptr<ClassLayer> class_layer = nullptr;
        if (with_class_layer) {
          class_layer = submap->getClassLayerPtr();
          class_layer->set_num_classes_to_serialize(k_num_counts);
        }

        for (int block_idx = 0; block_idx < k_num_blocks; block_idx++) {
          std::shared_ptr<voxblox::Block<voxblox::TsdfVoxel>> tsdf_block =
                  tsdf_layer->allocateBlockPtrByIndex(
                          voxblox::BlockIndex(block_idx, 0, 0));
          std::shared_ptr<voxblox::Block<ClassVoxelType>> class_block = nullptr;
          if (with_class_layer)
            class_block = class_layer->allocateBlockPtrByIndex(
                    voxblox::BlockIndex(block_idx, 0, 0));

          for (int i = 0; i < tsdf_block->num_voxels(); i++) {
            TsdfVoxel *tsdf_voxel = tsdf_block->getVoxelPtrByCoordinates(
                    tsdf_block->computeCoordinatesFromLinearIndex(i));
            ClassVoxelType *class_voxel = nullptr;
            if (with_class_layer)
              class_voxel = class_block->getVoxelPtrByCoordinates(
                      class_block->computeCoordinatesFromLinearIndex(i));

            load_random_voxels(tsdf_voxel, class_voxel);
          }
        }
      }
      return submap;
    }
}

TEST(SubmapSave, serializeSubmapWithClassLayer) {
  char* tmpname = strdup("/tmp/tmpfileXXXXXX");
  mkstemp(tmpname);

  std::unique_ptr<panoptic_mapping::Submap> to_save_submap = panoptic_mapping::getSubmap(true, true);
  std::fstream outfile;
  outfile.open(tmpname, std::fstream::out | std::fstream::binary);
  to_save_submap->saveToStream(&outfile);
  outfile.close();
  // Load file now
  std::ifstream infile;
  infile.open(tmpname, std::fstream::in | std::fstream::binary);
  std::unique_ptr<panoptic_mapping::Submap> to_load_submap = panoptic_mapping::getSubmap(false, true);
  uint64_t byte_offset = 0;
  to_load_submap = to_load_submap->loadFromStream(&infile, &byte_offset);

  infile.close();
  // Check equal

  std::pair<const panoptic_mapping::TsdfLayer, const panoptic_mapping::ClassLayer> saved_layers =
      std::pair<const panoptic_mapping::TsdfLayer, const panoptic_mapping::ClassLayer>(
          to_save_submap->getTsdfLayer(), to_save_submap->getClassLayer());
  std::pair<const panoptic_mapping::TsdfLayer, const panoptic_mapping::ClassLayer> loaded_layers =
      std::pair<const panoptic_mapping::TsdfLayer, const panoptic_mapping::ClassLayer>(
          to_load_submap->getTsdfLayer(), to_load_submap->getClassLayer());

  voxblox::BlockIndexList indexList;
  saved_layers.first.getAllAllocatedBlocks(&indexList);

  for (auto index : indexList) {
    // Check TSDF Layer
    panoptic_mapping::check_block_equal(saved_layers.first.getBlockPtrByIndex(index).get(),
                      loaded_layers.first.getBlockPtrByIndex(index).get());

    // Check class layer
    panoptic_mapping::check_block_equal(saved_layers.second.getBlockPtrByIndex(index).get(),
                      loaded_layers.second.getBlockPtrByIndex(index).get());
  }
}

TEST(SubmapSave, serializeSubmapWithoutClassLayer) {
  char* tmpname = strdup("/tmp/tmpfileXXXXXX");
  mkstemp(tmpname);

  std::unique_ptr<panoptic_mapping::Submap> to_save_submap = panoptic_mapping::getSubmap(true, false);

  std::fstream outfile;
  outfile.open(tmpname, std::fstream::out | std::fstream::binary);
  to_save_submap->saveToStream(&outfile);
  outfile.close();
  // Load file now
  std::ifstream infile;
  infile.open(tmpname, std::fstream::in | std::fstream::binary);
  std::unique_ptr<panoptic_mapping::Submap> to_load_submap = panoptic_mapping::getSubmap(false, false);
  uint64_t byte_offset = 0;
  to_load_submap = to_load_submap->loadFromStream(&infile, &byte_offset);
  infile.close();
  // Check equal
  voxblox::BlockIndexList indexList;
  to_save_submap->getTsdfLayerPtr()->getAllAllocatedBlocks(&indexList);
  for (auto index : indexList) {
    // Check TSDF Layer
    panoptic_mapping::check_block_equal(
        to_save_submap->getTsdfLayer().getBlockPtrByIndex(index).get(),
        to_load_submap->getTsdfLayer().getBlockPtrByIndex(index).get());
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}