#include "panoptic_mapping/map/classification/variable_count_weighted.h"

#include <memory>
#include <vector>

namespace panoptic_mapping {

ClassVoxelType VariableCountWeightedVoxel::getVoxelType() const {
  return ClassVoxelType::kVariableCountWeighted;
}

bool VariableCountWeightedVoxel::isObserverd() const {
  return !weights.empty();
}

bool VariableCountWeightedVoxel::belongsToSubmap() const {
  // In doubt we count the voxel as belonging. This also applies for unobserved
  // voxels.
  return current_index == 0;
}

float VariableCountWeightedVoxel::getBelongingProbability() const {
  return getProbability(0);
}

int VariableCountWeightedVoxel::getBelongingID() const { return current_index; }

float VariableCountWeightedVoxel::getProbability(const int id) const {
  if (weights.empty()) {
    return 0.f;
  }
  auto it = weights.find(id);
  if (it == weights.end()) {
    return 0.f;
  }
  return it->second / weight_sum;
}

void VariableCountWeightedVoxel::incrementCount(const int id,
                                                const float weight) {
  FloatingPoint& updated_weight = weights[id];
  updated_weight += weight;
  if (updated_weight > current_max_weight) {
    current_index = id;
    current_max_weight = updated_weight;
  }
  weight_sum += weight;
}

bool VariableCountWeightedVoxel::mergeVoxel(const ClassVoxel& other) {
  LOG(WARNING) << "Can not merge voxels of type VariableCountWeightedVoxel";
  return false;
}

std::vector<uint32_t> VariableCountWeightedVoxel::serializeVoxelToInt() const {
  // Store both id and weights as 32-bit uint
  std::vector<uint32_t> result(1 + weights.size() * 2);
  result[0] = weights.size();

  // Store all weights as id-value pair.
  size_t index = 0;
  for (const auto& id_weight_pair : weights) {
    index++;
    result[index] = static_cast<uint32_t>(id_weight_pair.first);
    index++;
    result[index] = int32FromX32(id_weight_pair.second);
  }
  return result;
}

bool VariableCountWeightedVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {
  if (*data_index >= data.size()) {
    LOG(WARNING)
        << "Can not deserialize voxel from integer data: Out of range (index: "
        << *data_index << ", data: " << data.size() << ")";
    return false;
  }

  // Check number of weights to load.
  const size_t length = data[*data_index];
  if (*data_index + 1 + 2 * length > data.size()) {
    LOG(WARNING) << "Can not deserialize voxel from integer data: Not enough "
                    "data (index: "
                 << *data_index << "-" << (*data_index + 1 + 2 * length)
                 << ", data: " << data.size() << ")";
    return false;
  }
  *data_index += 1;

  // Get the data.
  weights.clear();
  weight_sum = 0;
  current_index = -1;
  current_max_weight = 0;
  for (size_t i = 0; i < length; ++i) {
    int index = data[*data_index + i * 2];
    float weight = x32FromInt32<float>(data[*data_index + i * 2 + 1]);
    weights[index] = weight;
    weight_sum += weight;
    if (weight > current_max_weight) {
      current_max_weight = weight;
      current_index = index;
    }
  }
  *data_index += length * 2;
  return true;
}

config_utilities::Factory::RegistrationRos<
    ClassLayer, VariableCountWeightedLayer, float, int>
    VariableCountWeightedLayer::registration_("variable_count_weighted");

VariableCountWeightedLayer::VariableCountWeightedLayer(
    const Config& config, const float voxel_size, const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

ClassVoxelType VariableCountWeightedLayer::getVoxelType() const {
  return ClassVoxelType::kVariableCountWeighted;
}

std::unique_ptr<ClassLayer> VariableCountWeightedLayer::clone() const {
  return std::make_unique<VariableCountWeightedLayer>(*this);
}

std::unique_ptr<ClassLayer> VariableCountWeightedLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary weights.
  return std::make_unique<VariableCountWeightedLayer>(
      VariableCountWeightedLayer::Config(), submap_proto.voxel_size(),
      submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
