#include "panoptic_mapping/map/classification/binary_count.h"

#include <memory>
#include <vector>

namespace panoptic_mapping {

ClassVoxelType BinaryCountVoxel::getVoxelType() const {
  return ClassVoxelType::kBinaryCount;
}

bool BinaryCountVoxel::isObserverd() const {
  return belongs_count > 0u || foreign_count > 0u;
}

bool BinaryCountVoxel::belongsToSubmap() const {
  // In doubt we count the voxel as belonging. This also applies for unobserved
  // voxels.
  return belongs_count >= foreign_count;
}

float BinaryCountVoxel::getBelongingProbability() const {
  return static_cast<float>(belongs_count) /
         static_cast<float>(belongs_count + foreign_count);
}

int BinaryCountVoxel::getBelongingID() const {
  // 0 - belongin submap, 1 - else
  return foreign_count < belongs_count;
}

float BinaryCountVoxel::getProbability(const int id) const {
  return static_cast<float>(id == 0u ? belongs_count : foreign_count) /
         static_cast<float>(belongs_count + foreign_count);
}

void BinaryCountVoxel::incrementCount(const int id, const float weight) {
  // ID 0 is used for belonging voxels.
  if (id == 0u) {
    belongs_count++;
  } else {
    foreign_count++;
  }
}

std::vector<uint32_t> BinaryCountVoxel::serializeVoxelToInt() const {}

void BinaryCountVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {}

config_utilities::Factory::RegistrationRos<ClassLayer, BinaryCountLayer, float,
                                           int>
    BinaryCountLayer::registration_("binary_count");

BinaryCountLayer::BinaryCountLayer(const Config& config, const float voxel_size,
                                   const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

ClassVoxelType BinaryCountLayer::getVoxelType() const {
  return ClassVoxelType::kBinaryCount;
}

std::unique_ptr<ClassLayer> BinaryCountLayer::clone() const {
  return std::make_unique<BinaryCountLayer>(*this);
}

std::unique_ptr<ClassLayer> BinaryCountLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<BinaryCountLayer>(BinaryCountLayer::Config(),
                                            submap_proto.voxel_size(),
                                            submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
