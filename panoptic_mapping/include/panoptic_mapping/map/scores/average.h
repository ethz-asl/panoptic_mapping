#ifndef PANOPTIC_MAPPING_MAP_SCORES_AVERAGE_H_
#define PANOPTIC_MAPPING_MAP_SCORES_AVERAGE_H_

#include <memory>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/map/scores/score_layer_impl.h"
#include "panoptic_mapping/map/scores/score_voxel.h"

namespace panoptic_mapping {

/**
 * @brief Classification by counting the occurences of each label. The index 0
 * is generally reserved for the belonging submap by shifting all IDs by 1. The
 * memory for counting is lazily allocated since often only surface voxels are
 * relevant.
 */
struct AverageScoreVoxel : public ScoreVoxel {
 public:
  // Implement interfaces.
  ScoreVoxelType getVoxelType() const override;
  bool isObserverd() const override;
  float getScore() const override;
  void addMeasurement(const float score, const float weight = 1.f) override;
  bool mergeVoxel(const ScoreVoxel& other) override;
  std::vector<uint32_t> serializeVoxelToInt() const override;
  bool deseriliazeVoxelFromInt(const std::vector<uint32_t>& data,
                               size_t* data_index) override;
  // Data.
  float accumulated_weight = 0;
  float average_score;
};

class AverageScoreLayer : public ScoreLayerImpl<AverageScoreVoxel> {
 public:
  struct Config : public config_utilities::Config<Config> {
    Config() { setConfigName("AverageScoreLayer"); }

   protected:
    void fromRosParam() override {}
    void printFields() const override {}
  };

  AverageScoreLayer(const Config& config, const float voxel_size,
                    const int voxels_per_side);

  ScoreVoxelType getVoxelType() const override;
  std::unique_ptr<ScoreLayer> clone() const override;
  static std::unique_ptr<ScoreLayer> loadFromStream(
      const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
      uint64_t* /* tmp_byte_offset_ptr */);

 protected:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      ScoreLayer, AverageScoreLayer, float, int>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_SCORES_AVERAGE_H_
