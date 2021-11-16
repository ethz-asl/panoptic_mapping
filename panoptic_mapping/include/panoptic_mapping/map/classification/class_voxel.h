#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_VOXEL_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_VOXEL_H_

#include <vector>

namespace panoptic_mapping {

/**
 * @brief General interface for classification voxels. The classification voxels
 * are stored in classification layers and need to implement at least these
 * interfaces.
 */
struct ClassVoxel {
  virtual ~ClassVoxel() = default;

  /**
   * @brief Return true if the voxel has received any measurements yet.
   */
  virtual bool isObserverd() const = 0;

  /**
   * @brief Check if this voxel counts as belonging to the containing submap.
   */
  virtual bool belongsToSubmap() const = 0;

  /**
   * @brief Computes the probability or confidence that this voxel belongs to
   * the entity specified with the ID (submap, class, instance, ..., depending
   * on the employed classification scheme).
   *
   * @param id Entity ID to lookup.
   * @return float Probability in [0, 1] that this voxel belongs to the entity.
   */
  virtual float getBelongingProbability(const int id) const = 0;

  /**
   * @brief Increment the belief of this classification of belonging to the
   * entity specified with the ID (submap, class, instance, ..., depending on
   * the employed classification scheme).
   *
   * @param id Entity ID to increment.
   * @param weight Optional. The weight by which the count is to be increased if
   * supported by the employed classification scheme.
   */
  virtual void incrementCount(const int id, const float weight = 1.f) = 0;

  /**
   * @brief Serialization tool to serialize voxels and layers to integer data.
   *
   * @param data The current binary data to append the voxel to.
   */
  virtual void serializeVoxelToInt(std::vector<uint32_t>* data) const = 0;

  /**
   * @brief De-serialize the voxel from integer data.
   *
   * @param data The data to read from.
   * @param data_index Index from where to read the data. This index should be
   * updated to point to the end of the serialized data such that the next voxel
   * can be read from there.
   */
  virtual void deseriliazeVoxelFromInt(const std::vector<uint32_t>& data,
                                       size_t& data_index) = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_VOXEL_H_
