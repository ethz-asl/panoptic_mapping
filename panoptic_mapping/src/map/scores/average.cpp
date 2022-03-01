#include "panoptic_mapping/map/scores/average.h"

#include <vector>

namespace panoptic_mapping {

ScoreVoxelType AverageScoreVoxel::getVoxelType() const {
  return ScoreVoxelType::kAverage;
}

bool AverageScoreVoxel::isObserverd() const { return accumulated_weight != 0; }

float AverageScoreVoxel::getScore() const {
  if (!isObserverd()) {
    LOG(WARNING) << "Getting score of unabserved voxel";
  }
  return average_score;
}

void AverageScoreVoxel::addMeasurement(const float score, const float weight) {
  const float new_weight = accumulated_weight + weight;
  average_score = weight * score / new_weight +
                  accumulated_weight * average_score / new_weight;
  accumulated_weight = new_weight;
}

bool AverageScoreVoxel::mergeVoxel(const ScoreVoxel& other) {
  // Check type compatibility.
  auto voxel = dynamic_cast<const AverageScoreVoxel*>(&other);
  if (!voxel) {
    LOG(WARNING) << "Can not merge voxels that are not of same type "
                    "(AverageScoreVoxel).";
    return false;
  }
  addMeasurement(voxel->average_score, voxel->accumulated_weight);
  return true;
}

std::vector<uint32_t> AverageScoreVoxel::serializeVoxelToInt() const {
  std::vector<uint32_t> result(2u);
  result.push_back(int32FromX32<float>(accumulated_weight));
  result.push_back(int32FromX32<float>(average_score));
  return result;
}

bool AverageScoreVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {
  accumulated_weight = x32FromInt32<float>(data[*data_index]);
  average_score = x32FromInt32<float>(data[*data_index + 1]);
  *data_index += 2;
  return true;
}

config_utilities::Factory::RegistrationRos<ScoreLayer, AverageScoreLayer, float,
                                           int>
    AverageScoreLayer::registration_("average");

AverageScoreLayer::AverageScoreLayer(const Config& config,
                                     const float voxel_size,
                                     const int voxels_per_side)
    : config_(config.checkValid()),
      ScoreLayerImpl(voxel_size, voxels_per_side) {}

ScoreVoxelType AverageScoreLayer::getVoxelType() const {
  return ScoreVoxelType::kAverage;
}

std::unique_ptr<ScoreLayer> AverageScoreLayer::clone() const {
  return std::make_unique<AverageScoreLayer>(*this);
}

std::unique_ptr<ScoreLayer> AverageScoreLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<AverageScoreLayer>(AverageScoreLayer::Config(),
                                             submap_proto.voxel_size(),
                                             submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
