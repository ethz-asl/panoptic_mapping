#include "panoptic_mapping/map/classification/moving_binary_count.h"

#include <limits>
#include <memory>
#include <vector>

namespace panoptic_mapping {

ClassVoxelType MovingBinaryCountVoxel::getVoxelType() const {
  return ClassVoxelType::kMovingBinaryCount;
}

bool MovingBinaryCountVoxel::isObserverd() const {
  return belongs_count > 0u || foreign_count > 0u;
}

bool MovingBinaryCountVoxel::belongsToSubmap() const {
  // In doubt we count the voxel as belonging. This also applies for unobserved
  // voxels.
  return belongs_count >= foreign_count;
}

float MovingBinaryCountVoxel::getBelongingProbability() const {
  return static_cast<float>(belongs_count) /
         (static_cast<float>(belongs_count) +
          static_cast<float>(foreign_count));
}

int MovingBinaryCountVoxel::getBelongingID() const {
  // 0 - belongin submap, 1 - else
  return foreign_count < belongs_count;
}

float MovingBinaryCountVoxel::getProbability(const int id) const {
  return static_cast<float>(id == 0u ? belongs_count : foreign_count) /
         (static_cast<float>(belongs_count) +
          static_cast<float>(foreign_count));
}

void MovingBinaryCountVoxel::incrementCount(const int id, const float weight) {
  // ID 0 is used for belonging voxels.
  if (id == 0u) {
    if (belongs_count == std::numeric_limits<uint8_t>::max()) {
      belongs_count /= 2u;
      foreign_count /= 2u;
    }
    belongs_count++;
  } else {
    if (foreign_count == std::numeric_limits<uint8_t>::max()) {
      belongs_count /= 2u;
      foreign_count /= 2u;
    }
    foreign_count++;
  }
}

std::vector<uint32_t> MovingBinaryCountVoxel::serializeVoxelToInt() const {}

void MovingBinaryCountVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {}

config_utilities::Factory::RegistrationRos<ClassLayer, MovingBinaryCountLayer,
                                           float, int>
    MovingBinaryCountLayer::registration_("moving_binary_count");

MovingBinaryCountLayer::MovingBinaryCountLayer(const Config& config,
                                               const float voxel_size,
                                               const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

ClassVoxelType MovingBinaryCountLayer::getVoxelType() const {
  return ClassVoxelType::kMovingBinaryCount;
}

std::unique_ptr<ClassLayer> MovingBinaryCountLayer::clone() const {
  return std::make_unique<MovingBinaryCountLayer>(*this);
}

std::unique_ptr<ClassLayer> MovingBinaryCountLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<MovingBinaryCountLayer>(
      MovingBinaryCountLayer::Config(), submap_proto.voxel_size(),
      submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
