#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_BINARY_COUNTS_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_BINARY_COUNTS_H_

#include <memory>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/map/classification/class_layer_impl.h"
#include "panoptic_mapping/map/classification/class_voxel.h"

namespace panoptic_mapping {

/**
 * @brief Binary classification by simple counting, where ID 0 indicates the
 * voxel belongs.
 */
struct BinaryCountVoxel : public ClassVoxel {
 public:
  // Implement interfaces.
  ClassVoxelType getVoxelType() const override;
  bool isObserverd() const override;
  bool belongsToSubmap() const override;
  float getBelongingProbability() const override;
  int getBelongingID() const override;
  float getProbability(const int id) const override;
  void incrementCount(const int id, const float weight = 1.f) override;
  std::vector<uint32_t> serializeVoxelToInt() const override;
  void deseriliazeVoxelFromInt(const std::vector<uint32_t>& data,
                               size_t* data_index) override;
  // Data. uint16_t can store up to ~65k observations, so that should always be
  // sufficient.
  uint16_t belongs_count = 0;
  uint16_t foreign_count = 0;
};

class BinaryCountLayer : public ClassLayerImpl<BinaryCountVoxel> {
 public:
  struct Config : public config_utilities::Config<Config> {
    Config() { setConfigName("BinaryCountLayer"); }
  };

  BinaryCountLayer(const Config& config, const float voxel_size,
                   const size_t voxels_per_side);

  ClassVoxelType getVoxelType() const override;
  std::unique_ptr<ClassLayer> clone() const override;
  static std::unique_ptr<ClassLayer> loadFromStream(
      const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
      uint64_t* /* tmp_byte_offset_ptr */);

 protected:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      ClassLayer, BinaryCountLayer, float, size_t>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_BINARY_COUNTS_H_