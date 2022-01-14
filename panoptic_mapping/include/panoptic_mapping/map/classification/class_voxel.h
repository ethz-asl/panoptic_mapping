#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_VOXEL_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_VOXEL_H_

#include <vector>

namespace panoptic_mapping {

// Enumerate all implemented classification voxel types for objects that need to
// operate on specific voxel types.
enum class ClassVoxelType {
  kBinaryCount,
  kMovingBinaryCount,
  kFixedCount,
  kVariableCount,
  kUncertainty
};

/**
 * @brief General interface for classification voxels. The classification voxels
 * are stored in classification layers and need to implement at least these
 * interfaces.
 */
struct ClassVoxel {
  virtual ~ClassVoxel() = default;

  /**
   * @brief Expose the contained type of the voxel for objects that need to
   * operate on specific voxel types.
   */
  virtual ClassVoxelType getVoxelType() const = 0;

  /**
   * @brief Return true if the voxel has received any measurements yet.
   */
  virtual bool isObserverd() const = 0;

  /**
   * @brief Check if this voxel counts as belonging to the containing submap.
   */
  virtual bool belongsToSubmap() const = 0;

  /**
   * @brief Compute the probability in [0, 1] of this voxel belonging to the
   * containing submap.
   */
  virtual float getBelongingProbability() const = 0;

  /**
   * @brief Get the ID of entity (submap, class, instance, ..., depending
   * on the employed classification scheme) that this voxel is most likely to
   * belong to.
   */
  virtual int getBelongingID() const = 0;

  /**
   * @brief Computes the probability or confidence that this voxel belongs to
   * the entity specified with the ID (submap, class, instance, ..., depending
   * on the employed classification scheme).
   *
   * @param id Entity ID to lookup.
   * @return float Probability in [0, 1] that this voxel belongs to the entity.
   */
  virtual float getProbability(const int id) const = 0;

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
   * @brief Merge another voxel into this voxel. If the default merging
   * behavior is not desired, the voxel members are exposed to custom
   * manipulation.
   *
   * @param other Voxel to merge into this one.
   * @return True if the merging was successful.
   */
  virtual bool mergeVoxel(const ClassVoxel& other) = 0;

  /**
   * @brief Serialization tool to serialize voxels and layers to integer data.
   *
   * @param data The current binary data to append the voxel to.
   */
  virtual std::vector<uint32_t> serializeVoxelToInt() const = 0;

  /**
   * @brief De-serialize the voxel from integer data.
   *
   * @param data The data to read from.
   * @param data_index Index from where to read the data. This index should be
   * updated to point to the end of the serialized data such that the next voxel
   * can be read from there.
   * @return True if the voxel was correctly de-serialized, false otherwise.
   */
  virtual bool deseriliazeVoxelFromInt(const std::vector<uint32_t>& data,
                                       size_t* data_index) = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_VOXEL_H_
