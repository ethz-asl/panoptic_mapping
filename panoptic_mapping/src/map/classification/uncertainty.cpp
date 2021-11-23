#include "panoptic_mapping/map/classification/uncertainty.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace panoptic_mapping {

ClassVoxelType UncertaintyVoxel::getVoxelType() const {
  return ClassVoxelType::kUncertainty;
}

std::vector<uint32_t> UncertaintyVoxel::serializeVoxelToInt() const {}

void UncertaintyVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {}

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
