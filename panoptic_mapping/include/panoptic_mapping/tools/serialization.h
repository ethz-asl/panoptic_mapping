/**
 * Functions that implement voxel to int32 de-/serialization.
 * In order to save memory, only the top
 * <PANOPTIC_MAPPING_SERIALIZE_TOP_N_COUNTS> class assignements are serialized.
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

// Store only top 3 counts per voxel to save memory
#define PANOPTIC_MAPPING_SERIALIZE_TOP_N_COUNTS uint8_t(3)

#ifdef PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
#define COUNTER_SIZE_BITS int(8)
#else
#define COUNTER_SIZE_BITS int(16)
#endif

namespace voxblox {

/**
 * Serializes a given Block to uint32_t
 * @param data vector of uint32_t where serialized integers should be stored
 */
template <>
void Block<panoptic_mapping::ClassVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const;

/**
 *  Serializes a given Block to uint32_t
 * @param data vector of uint32_t where serialized integers should be stored
 */
template <>
void Block<panoptic_mapping::ClassUncertaintyVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const;

/**
 * Deserializes a given Block from uint32_t data
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

template <>
void mergeVoxelAIntoVoxelB(const panoptic_mapping::ClassVoxel& voxel_A,
                           panoptic_mapping::ClassVoxel* voxel_B);

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
