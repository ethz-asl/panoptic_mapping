#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_FIXED_COUNT_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_FIXED_COUNT_H_

#include <memory>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/map/classification/class_layer_impl.h"
#include "panoptic_mapping/map/classification/class_voxel.h"

namespace panoptic_mapping {

/**
 * @brief Classification by counting the occurences of each label. The index 0
 * is generally reserved for the belonging submap by shifting all IDs by 1. The
 * memory for counting is lazily allocated since often only surface voxels are
 * relevant.
 */
struct FixedCountVoxel : public ClassVoxel {
 public:
  // Implement interfaces.
  ClassVoxelType getVoxelType() const override;
  bool isObserverd() const override;
  bool belongsToSubmap() const override;
  float getBelongingProbability() const override;
  int getBelongingID() const override;
  float getProbability(const int id) const override;
  void incrementCount(const int id, const float weight = 1.f) override;
  bool mergeVoxel(const ClassVoxel& other) override;
  std::vector<uint32_t> serializeVoxelToInt() const override;
  bool deseriliazeVoxelFromInt(const std::vector<uint32_t>& data,
                               size_t* data_index) override;
  // Data.
  std::vector<ClassificationCount> counts;
  int current_index = 0;
  ClassificationCount current_count = 0;
  ClassificationCount total_count = 0;

  static size_t numCounts();
  static void setNumCounts(size_t num_counts);

 private:
  // Fixed count voxels store a fixed number of labels, which is currently set
  // via this global setting.
  static size_t kNumCounts;
};

class FixedCountLayer : public ClassLayerImpl<FixedCountVoxel> {
 public:
  struct Config : public config_utilities::Config<Config> {
    Config() { setConfigName("FixedCountLayer"); }

   protected:
    void fromRosParam() override {}
    void printFields() const override {}
  };

  FixedCountLayer(const Config& config, const float voxel_size,
                  const int voxels_per_side);

  ClassVoxelType getVoxelType() const override;
  std::unique_ptr<ClassLayer> clone() const override;
  static std::unique_ptr<ClassLayer> loadFromStream(
      const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
      uint64_t* /* tmp_byte_offset_ptr */);

 protected:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<ClassLayer, FixedCountLayer,
                                                    float, int>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_FIXED_COUNT_H_
