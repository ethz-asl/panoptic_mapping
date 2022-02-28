#include "panoptic_mapping/map/scores/latest.h"

#include <vector>

namespace panoptic_mapping {

ScoreVoxelType LatestScoreVoxel::getVoxelType() const {
  return ScoreVoxelType::kLatest;
}

bool LatestScoreVoxel::isObserverd() const { return observed; }
float LatestScoreVoxel::getScore() const { return latest_score; }
void LatestScoreVoxel::addMeasurement(const float score, const float weight) {
  observed = true;
  latest_score = score;
}

bool LatestScoreVoxel::mergeVoxel(const ScoreVoxel& other) {
  // Check type compatibility.
  auto voxel = dynamic_cast<const LatestScoreVoxel*>(&other);
  if (!voxel) {
    LOG(WARNING)
        << "Can not merge voxels that are not of same type (LatestScoreVoxel).";
    return false;
  }
  // both not observed -> no changes
  if (!observed && !voxel->observed) {
    return true;
  }
  // both are observed -> we default to this voxel
  if (observed && voxel->observed) {
    return true;
  }
  // only one is observed, take its value
  if (voxel->observed) {
    addMeasurement(voxel->latest_score);
  }
  return true;
}

std::vector<uint32_t> LatestScoreVoxel::serializeVoxelToInt() const {
  std::vector<uint32_t> result(2u);
  result.push_back(static_cast<uint32_t>(observed));
  result.push_back(int32FromX32<float>(latest_score));
  return result;
}

bool LatestScoreVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {
  observed = data[*data_index];
  latest_score = x32FromInt32<float>(data[*data_index + 1]);
  *data_index += 2;
  return true;
}

config_utilities::Factory::RegistrationRos<ScoreLayer, LatestScoreLayer, float,
                                           int>
    LatestScoreLayer::registration_("latest");

LatestScoreLayer::LatestScoreLayer(const Config& config,
                                     const float voxel_size,
                                     const int voxels_per_side)
    : config_(config.checkValid()),
      ScoreLayerImpl(voxel_size, voxels_per_side) {}

ScoreVoxelType LatestScoreLayer::getVoxelType() const {
  return ScoreVoxelType::kLatest;
}

std::unique_ptr<ScoreLayer> LatestScoreLayer::clone() const {
  return std::make_unique<LatestScoreLayer>(*this);
}

std::unique_ptr<ScoreLayer> LatestScoreLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<LatestScoreLayer>(LatestScoreLayer::Config(),
                                             submap_proto.voxel_size(),
                                             submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
