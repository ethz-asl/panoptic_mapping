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
std::vector<uint32_t> VariableCountVoxel::serializeVoxelToInt() const {}

void VariableCountVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {}

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
