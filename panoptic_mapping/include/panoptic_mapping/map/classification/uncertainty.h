#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_UNCERTAINTY_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_UNCERTAINTY_H_

#include <memory>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/map/classification/class_layer_impl.h"
#include "panoptic_mapping/map/classification/class_voxel.h"
#include "panoptic_mapping/map/classification/fixed_count.h"

namespace panoptic_mapping {

/**
 * @brief Specialization of the fixed count voxel that additionally stores an
 * aggregated uncertainty and can be labeled as groudn truth.
 */
struct UncertaintyVoxel : public FixedCountVoxel {
 public:
  // Implement interfaces.
  ClassVoxelType getVoxelType() const override;
  bool mergeVoxel(const ClassVoxel& other) override;
  std::vector<uint32_t> serializeVoxelToInt() const override;
  bool deseriliazeVoxelFromInt(const std::vector<uint32_t>& data,
                               size_t* data_index) override;
  // Data.
  float uncertainty = 0.f;
  bool is_ground_truth = false;
};

class UncertaintyLayer : public ClassLayerImpl<UncertaintyVoxel> {
 public:
  struct Config : public config_utilities::Config<Config> {
    Config() { setConfigName("UncertaintyLayer"); }

   protected:
    void fromRosParam() override {}
    void printFields() const override {}
  };

  UncertaintyLayer(const Config& config, const float voxel_size,
                   const int voxels_per_side);

  ClassVoxelType getVoxelType() const override;
  std::unique_ptr<ClassLayer> clone() const override;
  static std::unique_ptr<ClassLayer> loadFromStream(
      const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
      uint64_t* /* tmp_byte_offset_ptr */);

 protected:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      ClassLayer, UncertaintyLayer, float, int>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_UNCERTAINTY_H_
