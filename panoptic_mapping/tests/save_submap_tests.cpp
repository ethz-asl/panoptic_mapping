#include <cstdlib>
#include <memory>
#include <queue>
#include <utility>

#include <gtest/gtest.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/tests/test_utils.h"

const int kNumBlocks = 10;

/**
 * Returns a random submap containing class and tsdf voxels.
 * The voxels are calculated randomly thus they will not make any sense. Do not
 * try to visualize this map or extract meshes
 * @return submap ptr
 */

// TODO
std::unique_ptr<panoptic_mapping::SubmapCollection> getSubmapCollection(
    bool fill, bool with_class_layer) {
  panoptic_mapping::Submap::Config cfg;
  cfg.use_class_layer = with_class_layer;

  auto submap_collection =
      std::make_unique<panoptic_mapping::SubmapCollection>();
  auto submap = submap_collection->createSubmap(cfg);

  if (fill) {
    auto tsdf_layer = submap->getTsdfLayerPtr();
    std::shared_ptr<panoptic_mapping::ClassLayer> class_layer = nullptr;
    if (with_class_layer) {
      class_layer = submap->getClassLayerPtr();
      class_layer->setNumClassesToSerialize(top_n_to_serialize);
    }

    for (int block_idx = 0; block_idx < kNumBlocks; block_idx++) {
      std::shared_ptr<voxblox::Block<voxblox::TsdfVoxel>> tsdf_block =
          tsdf_layer->allocateBlockPtrByIndex(
              voxblox::BlockIndex(block_idx, 0, 0));
      std::shared_ptr<voxblox::Block<panoptic_mapping::ClassVoxel>>
          class_block = nullptr;
      if (with_class_layer)
        class_block = class_layer->allocateBlockPtrByIndex(
            voxblox::BlockIndex(block_idx, 0, 0));

      for (int i = 0; i < tsdf_block->num_voxels(); i++) {
        panoptic_mapping::TsdfVoxel* tsdf_voxel =
            tsdf_block->getVoxelPtrByCoordinates(
                tsdf_block->computeCoordinatesFromLinearIndex(i));
        panoptic_mapping::ClassVoxel* class_voxel = nullptr;
        if (with_class_layer)
          class_voxel = class_block->getVoxelPtrByCoordinates(
              class_block->computeCoordinatesFromLinearIndex(i));

        loadRandomVoxels(tsdf_voxel, class_voxel);
      }
    }
  }
  return submap_collection;
}

void checkSaveAndLoadCollection(
    std::unique_ptr<panoptic_mapping::SubmapCollection> to_save_collection,
    std::unique_ptr<panoptic_mapping::SubmapCollection> to_load_collection) {
  char* tmpname = strdup("/tmp/tmpfileXXXXXX");
  mkstemp(tmpname);
  to_save_collection->saveToFile(tmpname);
  to_load_collection->loadFromFile(tmpname);
  // Check equal
  auto to_save_submap = to_save_collection->getSubmapPtr(0);
  auto to_load_submap = to_load_collection->getSubmapPtr(0);

  std::pair<const panoptic_mapping::TsdfLayer,
            const panoptic_mapping::ClassLayer>
      saved_layers = std::pair<const panoptic_mapping::TsdfLayer,
                               const panoptic_mapping::ClassLayer>(
          to_save_submap->getTsdfLayer(), to_save_submap->getClassLayer());
  std::pair<const panoptic_mapping::TsdfLayer,
            const panoptic_mapping::ClassLayer>
      loaded_layers = std::pair<const panoptic_mapping::TsdfLayer,
                                const panoptic_mapping::ClassLayer>(
          to_load_submap->getTsdfLayer(), to_load_submap->getClassLayer());

  voxblox::BlockIndexList indexList;
  saved_layers.first.getAllAllocatedBlocks(&indexList);

  for (auto index : indexList) {
    // Check TSDF Layer
    checkBlockEqual(saved_layers.first.getBlockPtrByIndex(index).get(),
                    loaded_layers.first.getBlockPtrByIndex(index).get());

    // Check class layer
    checkBlockEqual(saved_layers.second.getBlockPtrByIndex(index).get(),
                    loaded_layers.second.getBlockPtrByIndex(index).get());
  }
}

TEST(SubmapSave, serializeSubmapWithClassLayer) {
  checkSaveAndLoadCollection(getSubmapCollection(true, true),
                             getSubmapCollection(false, true));
}
TEST(SubmapSave, serializeSubmapWithoutClassLayer) {
  checkSaveAndLoadCollection(getSubmapCollection(true, false),
                             getSubmapCollection(false, false));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
