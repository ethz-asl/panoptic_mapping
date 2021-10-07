#ifndef PANOPTIC_MAPPING_COMMON_CLASS_VOXEL_LAYER_H_
#define PANOPTIC_MAPPING_COMMON_CLASS_VOXEL_LAYER_H_

#include <voxblox/core/layer.h>

namespace panoptic_mapping {
template <typename VoxelType>
class ClassVoxelLayer : public voxblox::Layer<VoxelType> {
  // How many class assignements should be serialized.
  int serialize_top_n_classes_ = 4;

 public:
  explicit ClassVoxelLayer<VoxelType>(voxblox::FloatingPoint voxel_size,
                                      size_t voxels_per_side)
      : voxblox::Layer<VoxelType>(voxel_size, voxels_per_side) {}

  explicit ClassVoxelLayer<VoxelType>(voxblox::FloatingPoint voxel_size,
                                      size_t voxels_per_side,
                                      int serialize_top_n_classes)
      : voxblox::Layer<VoxelType>(voxel_size, voxels_per_side),
        serialize_top_n_classes_(serialize_top_n_classes) {}

  bool saveBlocksToStream(bool include_all_blocks,
                          voxblox::BlockIndexList blocks_to_include,
                          std::fstream* outfile_ptr) const;

  void set_num_classes_to_serialize(int serialize_top_n_classes) {
    serialize_top_n_classes_ = serialize_top_n_classes;
  }
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_CLASS_VOXEL_LAYER_H_
