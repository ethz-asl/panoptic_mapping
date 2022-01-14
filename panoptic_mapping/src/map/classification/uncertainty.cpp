#include "panoptic_mapping/map/classification/uncertainty.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace panoptic_mapping {

ClassVoxelType UncertaintyVoxel::getVoxelType() const {
  return ClassVoxelType::kUncertainty;
}

bool UncertaintyVoxel::mergeVoxel(const ClassVoxel& other) {
  // Check type compatibility.
  auto voxel = dynamic_cast<const UncertaintyVoxel*>(&other);
  if (!voxel) {
    LOG(WARNING)
        << "Can not merge voxels that are not of same type (UncertaintyVoxel).";
    return false;
  }
  if (is_ground_truth) {
    return true;
  }
  if (voxel->is_ground_truth) {
    counts = voxel->counts;
    current_index = voxel->current_index;
    current_count = voxel->current_count;
    uncertainty = voxel->uncertainty;
    is_ground_truth = true;
    return true;
  }
  FixedCountVoxel::mergeVoxel(other);
  // We assume that uncertainties can be averaged here.
  uncertainty = (uncertainty + voxel->uncertainty) / 2.f;
  return true;
}

std::vector<uint32_t> UncertaintyVoxel::serializeVoxelToInt() const {
  // Serialize the count data.
  std::vector<uint32_t> result = FixedCountVoxel::serializeVoxelToInt();

  // Append the added data.
  result.push_back(static_cast<uint32_t>(is_ground_truth));
  result.push_back(int32FromX32<float>(uncertainty));
  return result;
}

bool UncertaintyVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {
  // De-serialize count data.
  FixedCountVoxel::deseriliazeVoxelFromInt(data, data_index);
  if (*data_index + 1 >= data.size()) {
    LOG(WARNING)
        << "Can not deserialize voxel from integer data: Out of range (index: "
        << *data_index << ", data: " << data.size() << ")";
    return false;
  }

  // De-serialize the added data.
  is_ground_truth = data[*data_index];
  uncertainty = x32FromInt32<float>(data[*data_index + 1]);
  *data_index += 2;
  return true;
}

config_utilities::Factory::RegistrationRos<ClassLayer, UncertaintyLayer, float,
                                           int>
    UncertaintyLayer::registration_("uncertainty");

UncertaintyLayer::UncertaintyLayer(const Config& config, const float voxel_size,
                                   const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

ClassVoxelType UncertaintyLayer::getVoxelType() const {
  return ClassVoxelType::kUncertainty;
}

std::unique_ptr<ClassLayer> UncertaintyLayer::clone() const {
  return std::make_unique<UncertaintyLayer>(*this);
}

std::unique_ptr<ClassLayer> UncertaintyLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<UncertaintyLayer>(UncertaintyLayer::Config(),
                                            submap_proto.voxel_size(),
                                            submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
