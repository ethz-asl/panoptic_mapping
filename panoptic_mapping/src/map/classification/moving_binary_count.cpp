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

bool MovingBinaryCountVoxel::mergeVoxel(const ClassVoxel& other) {
  // Check type compatibility.
  auto voxel = dynamic_cast<const MovingBinaryCountVoxel*>(&other);
  if (!voxel) {
    LOG(WARNING) << "Can not merge voxels that are not of same type "
                    "(MovingBinaryCountVoxel).";
    return false;
  }
  const int new_belongs_count =
      static_cast<int>(belongs_count) + static_cast<int>(voxel->belongs_count);
  const int new_foreign_count =
      static_cast<int>(foreign_count) + static_cast<int>(voxel->foreign_count);
  const int normalization =
      (new_belongs_count > 255 || new_foreign_count > 255) ? 2 : 1;
  belongs_count =
      static_cast<ClassificationCount>(new_belongs_count / normalization);
  foreign_count =
      static_cast<ClassificationCount>(new_foreign_count / normalization);
  return true;
}

std::vector<uint32_t> MovingBinaryCountVoxel::serializeVoxelToInt() const {
  // Pack both values into an uint16 and return as uint32, will be further
  // packed by the layer.
  return {int16FromTwoInt8(belongs_count, foreign_count)};
}

bool MovingBinaryCountVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {
  // Since the a voxel only needs half a word advance the index in half steps.
  if (*data_index >= data.size()) {
    LOG(WARNING)
        << "Can not deserialize voxel from integer data: Out of range (index: "
        << *data_index << ", data: " << data.size() << ")";
    return false;
  }

  // Get the data. The layer stores only uint16 in the data.
  const std::pair<uint8_t, uint8_t> counts =
      twoInt8FromInt16(data[*data_index]);
  belongs_count = counts.first;
  foreign_count = counts.second;
  (*data_index)++;
  return true;
}

config_utilities::Factory::RegistrationRos<ClassLayer, MovingBinaryCountLayer,
                                           float, int>
    MovingBinaryCountLayer::registration_("moving_binary_count");

MovingBinaryCountLayer::MovingBinaryCountLayer(const Config& config,
                                               const float voxel_size,
                                               const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

bool MovingBinaryCountLayer::saveBlockToStream(
    BlockIndex block_index, std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  auto block = layer_.getBlockPtrByIndex(block_index);
  if (!block) {
    return false;
  }

  // Save data.
  voxblox::BlockProto proto;
  proto.set_has_data(block->has_data());
  proto.set_voxels_per_side(block->voxels_per_side());
  proto.set_voxel_size(block->voxel_size());
  proto.set_origin_x(block->origin().x());
  proto.set_origin_y(block->origin().y());
  proto.set_origin_z(block->origin().z());
  uint16_t tmp_data;
  bool first_packet_half = true;
  int test = 0;
  for (size_t i = 0; i < block->num_voxels(); ++i) {
    const uint16_t data = static_cast<uint16_t>(
        block->getVoxelByLinearIndex(i).serializeVoxelToInt()[0]);
    // Always combine two voxels into a word. The number of voxels is always a
    // multiple of two.
    if (first_packet_half) {
      tmp_data = data;
      first_packet_half = false;
    } else {
      proto.add_voxel_data(int32FromTwoInt16(tmp_data, data));
      first_packet_half = true;
    }
  }
  if (!voxblox::utils::writeProtoMsgToStream(proto, outfile_ptr)) {
    LOG(ERROR) << "Could not write class block proto message to stream.";
    return false;
  }
  return true;
}

bool MovingBinaryCountLayer::addBlockFromProto(
    const voxblox::BlockProto& block_proto) {
  // Check compatibility.
  if (!isCompatible(block_proto, *this)) {
    return false;
  }

  // Add (potentially replace) the block.
  const Point origin(block_proto.origin_x(), block_proto.origin_y(),
                     block_proto.origin_z());
  layer_.removeBlockByCoordinates(origin);
  auto block = layer_.allocateNewBlockByCoordinates(origin);

  // Read the data, where two voxels are unpacked from each word.
  std::vector<uint32_t> data;
  data.resize(block_proto.voxel_data_size() * 2);
  size_t index = 0;
  for (uint32_t word : block_proto.voxel_data()) {
    const std::pair<uint16_t, uint16_t> datum = twoInt16FromInt32(word);
    data[index] = datum.first;
    data[index + 1] = datum.second;
    index += 2;
  }

  // Load the voxels.
  index = 0;
  for (size_t i = 0; i < block->num_voxels(); ++i) {
    if (!reinterpret_cast<ClassVoxel&>(block->getVoxelByLinearIndex(i))
             .deseriliazeVoxelFromInt(data, &index)) {
      LOG(WARNING) << "Could not serialize voxel from data.";
      return false;
    }
  }
  return true;
}

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
