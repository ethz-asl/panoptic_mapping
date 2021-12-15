#include "panoptic_mapping/map/classification/variable_count.h"

#include <memory>
#include <vector>

namespace panoptic_mapping {

ClassVoxelType VariableCountVoxel::getVoxelType() const {
  return ClassVoxelType::kVariableCount;
}

bool VariableCountVoxel::isObserverd() const { return !counts.empty(); }

bool VariableCountVoxel::belongsToSubmap() const {
  // In doubt we count the voxel as belonging. This also applies for unobserved
  // voxels.
  return current_index == 0;
}

float VariableCountVoxel::getBelongingProbability() const {
  return getProbability(0);
}

int VariableCountVoxel::getBelongingID() const { return current_index; }

float VariableCountVoxel::getProbability(const int id) const {
  if (counts.empty()) {
    return 0.f;
  }
  auto it = counts.find(id);
  if (it == counts.end()) {
    return 0.f;
  }
  return static_cast<float>(it->second) / static_cast<float>(total_count);
}

void VariableCountVoxel::incrementCount(const int id, const float weight) {
  const ClassificationCount new_count = ++(counts[id]);
  if (new_count > current_count) {
    current_index = id;
    current_count = new_count;
  }
  ++total_count;
}

bool VariableCountVoxel::mergeVoxel(const ClassVoxel& other) {
  // Check type compatibility.
  auto voxel = dynamic_cast<const VariableCountVoxel*>(&other);
  if (!voxel) {
    LOG(WARNING) << "Can not merge voxels that are not of same type "
                    "(VariableCountVoxel).";
    return false;
  }
  // No averaging is performed here. This inflates the number of total counts
  // but keeps the accuracy higher.
  for (const auto& id_count_pair : voxel->counts) {
    total_count += id_count_pair.second;
    counts[id_count_pair.first] += id_count_pair.second;
    if (counts[id_count_pair.first] > current_count) {
      current_count = counts[id_count_pair.first];
      current_index = id_count_pair.first;
    }
  }
  return true;
}

std::vector<uint32_t> VariableCountVoxel::serializeVoxelToInt() const {
  // To save memory, here we just assume that the IDs stored in the map are in
  // int_16 range (-32k:32k).
  std::vector<uint32_t> result(counts.size() + 1);
  result[0] = counts.size();

  // Store all counts as id-value pair.
  size_t index = 0;
  for (const auto& id_count_pair : counts) {
    index++;
    if (id_count_pair.first < std::numeric_limits<int16_t>::lowest() ||
        id_count_pair.first > std::numeric_limits<int16_t>::max()) {
      LOG(WARNING) << "ID: '" << id_count_pair.first
                   << "' is out of Int16 range and will be ignored.";
      continue;
    }
    result[index] = int32FromTwoInt16(
        static_cast<uint16_t>(id_count_pair.first), id_count_pair.second);
  }
  return result;
}

bool VariableCountVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {
  if (*data_index >= data.size()) {
    LOG(WARNING)
        << "Can not deserialize voxel from integer data: Out of range (index: "
        << *data_index << ", data: " << data.size() << ")";
    return false;
  }

  // Check number of counts to load.
  const size_t length = data[*data_index] + 1;
  if (*data_index + length > data.size()) {
    LOG(WARNING) << "Can not deserialize voxel from integer data: Not enough "
                    "data (index: "
                 << *data_index << "-" << (*data_index + length)
                 << ", data: " << data.size() << ")";
    return false;
  }

  // Get the data.
  counts.clear();
  total_count = 0;
  current_index = -1;
  current_count = 0;
  for (size_t i = 1; i < length; ++i) {
    std::pair<uint16_t, uint16_t> datum =
        twoInt16FromInt32(data[*data_index + i]);
    counts[static_cast<int16_t>(datum.first)] = datum.second;
    total_count += datum.second;
    if (datum.second > current_count) {
      current_count = datum.second;
      current_index = datum.first;
    }
  }
  *data_index += length;
  return true;
}

config_utilities::Factory::RegistrationRos<ClassLayer, VariableCountLayer,
                                           float, int>
    VariableCountLayer::registration_("variable_count");

VariableCountLayer::VariableCountLayer(const Config& config,
                                       const float voxel_size,
                                       const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

ClassVoxelType VariableCountLayer::getVoxelType() const {
  return ClassVoxelType::kVariableCount;
}

std::unique_ptr<ClassLayer> VariableCountLayer::clone() const {
  return std::make_unique<VariableCountLayer>(*this);
}

std::unique_ptr<ClassLayer> VariableCountLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<VariableCountLayer>(VariableCountLayer::Config(),
                                              submap_proto.voxel_size(),
                                              submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
