/**
 * Functions that implement voxel to int32 de-/serialization.
 * In order to save memory, only the top N (parameter) class assignements are serialized.
 * Most of the times, voxel are only assigned 3-4 Classes, and there is not much
 * precision lost by not saving the others.
 *
 * The class counts are serialized by first encoding the indices of the top N
 * classes and then the respective counts If the number of counts is the same as
 * the number of classes, this method uses more memory than simply storing the
 * whole count vector, since the indices are also saved.
 */
#ifndef PANOPTIC_MAPPING_TOOLS_SERIALIZATION_H
#define PANOPTIC_MAPPING_TOOLS_SERIALIZATION_H
#include "panoptic_mapping/common/common.h"

// How many class assignements per voxel should be serialized if no parameter was provided
#define PANOPTIC_MAPPING_DEFAULT_SERIALIZATION_COUNT int(4)

#ifdef PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
#define COUNTER_SIZE_BITS int(8)
#else
#define COUNTER_SIZE_BITS int(16)
#endif

namespace voxblox {

/**
 * Converts an int32 vector into a ClassVoxel
 */
bool readVoxelFromInt32(const std::vector<uint32_t>* data, size_t& data_idx,
                        int serialize_top_n_counts,
                        panoptic_mapping::ClassVoxel& voxel);

/**
 * Converts an int32 vector into a ClassUncertaintyVoxel
 */
bool readVoxelFromInt32(const std::vector<uint32_t>* data, size_t& data_idx,
                        int serialize_top_n_counts,
                        panoptic_mapping::ClassUncertaintyVoxel& voxel);

/**
 * Converts a Class Voxel into an int32 vector
 */
bool convertVoxelToInt32(const panoptic_mapping::ClassVoxel& voxel,
                         std::vector<uint32_t>* data,
                         int serialize_top_n_counts);

/**
 * Converts a Class Voxel into an int32 vector
 */
bool convertVoxelToInt32(const panoptic_mapping::ClassUncertaintyVoxel& voxel,
                         std::vector<uint32_t>* data,
                         int serialize_top_n_counts);

/**
 * Deserializes a given Block from uint32_t data.
 * @param data
 */
template <>
void Block<panoptic_mapping::ClassVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data);

/**
 * Deserializes a given Block from uint32_t data
 * @param data
 */
template <>
void Block<panoptic_mapping::ClassUncertaintyVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data);

/**
 * This function is currently not implemented and only exists so voxblox can compile
 */
template <>
void mergeVoxelAIntoVoxelB(const panoptic_mapping::ClassVoxel& voxel_A,
                           panoptic_mapping::ClassVoxel* voxel_B);

/**
 * Wrapper to serialize a Block to int32.
 * If possible use serializeBlockToIntegers(
 *   const Block<VoxelType>* block, std::vector<uint32_t>* data,
 *   int serialize_top_n_counts) instead.
 */
template <typename VoxelType>
void serializeBlockToIntegers(const Block<VoxelType>* block,
                              std::vector<uint32_t>* data,
                              int serialize_top_n_counts) {
  CHECK_NOTNULL(data);
  // Make sure 32 is a multiple of COUNTER_SIZE_BITS
  CHECK_EQ(32 % COUNTER_SIZE_BITS, 0);
  data->clear();
  data->push_back(static_cast<uint32_t>(serialize_top_n_counts));
  // Save all voxels to uint32.
  // NOTE @zrene size of encoded list depends on class voxel. If no class was
  // assigned, it will have size 1 This means static size checking is not
  // possible.
  for (size_t voxel_idx = 0u; voxel_idx < block->num_voxels(); ++voxel_idx) {
    convertVoxelToInt32(block->getVoxelByLinearIndex(voxel_idx), data,
                    serialize_top_n_counts);  // Default implementation so
    // voxblox does not break
  }
}

template <typename VoxelType>
void deserializeBlockFromIntegers(Block<VoxelType>* block,
                                  const std::vector<uint32_t>& data) {
  // Make sure 32 is a multiple of COUNTER_SIZE_BITS
  CHECK_EQ(32 % COUNTER_SIZE_BITS, 0);
  const size_t num_data_packets = data.size();
  // Define outside to check if end has been reached after loop
  size_t voxel_idx = 0u, data_idx = 0u;
  int deserialize_top_n_counts = static_cast<int>(data[data_idx++]);

  for (; voxel_idx < block->num_voxels() && data_idx < num_data_packets;
         ++voxel_idx) {
    readVoxelFromInt32(&data, data_idx, deserialize_top_n_counts,
                          block->getVoxelByLinearIndex(voxel_idx));
  }
  // Make sure all voxels have been loaded
  CHECK_EQ(voxel_idx, block->num_voxels());
  // Make sure all ints have been read
  CHECK_EQ(data_idx, num_data_packets);
}

// Parsing Proto
template <typename VoxelType>
void getProtoForBlock(const Block<VoxelType>* block, BlockProto* proto,
                      int serialize_top_n_counts) {
  CHECK_NOTNULL(proto);
  proto->set_voxels_per_side(block->voxels_per_side());
  proto->set_voxel_size(block->voxel_size());

  proto->set_origin_x(block->origin().x());
  proto->set_origin_y(block->origin().y());
  proto->set_origin_z(block->origin().z());

  proto->set_has_data(block->has_data());

  std::vector<uint32_t> data;
  serializeBlockToIntegers<VoxelType>(block, &data, serialize_top_n_counts);

  for (uint32_t word : data) {
    proto->add_voxel_data(word);
  }
}

// Other Needed functions

/**
 * New VoxelType for this kind off voxels
 */
namespace voxel_types {
const std::string kClass = "class";
}  // namespace voxel_types

template <>
std::string getVoxelType<panoptic_mapping::ClassVoxel>();

template <>
std::string getVoxelType<panoptic_mapping::ClassUncertaintyVoxel>();
}  // namespace voxblox

#endif  // PANOPTIC_MAPPING_TOOLS_SERIALIZATION_H
