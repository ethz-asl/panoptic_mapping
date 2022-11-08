#ifndef PANOPTIC_MAPPING_MAP_SCORES_SCORE_VOXEL_H_
#define PANOPTIC_MAPPING_MAP_SCORES_SCORE_VOXEL_H_

#include <vector>

namespace panoptic_mapping {

// Enumerate all implemented classification voxel types for objects that need to
// operate on specific voxel types.
enum class ScoreVoxelType {
  kLatest,
  kAverage,
};

/**
 * @brief General interface for voxels that aggregate a floating point number,
 * e.g. an uncertainty, novelty score or the likes.
 */
struct ScoreVoxel {
  virtual ~ScoreVoxel() = default;

  /**
   * @brief Expose the contained type of the voxel for objects that need to
   * operate on specific voxel types.
   */
  virtual ScoreVoxelType getVoxelType() const = 0;

  /**
   * @brief Return true if the voxel has received any measurements yet.
   */
  virtual bool isObserverd() const = 0;

  /**
   * @brief Return the current score value in the voxel.
   * @return float The score.
   */
  virtual float getScore() const = 0;

  /**
   * @brief Add a measurement to this voxel that will be aggregated dependent on
   * the specified voxel type.
   *
   * @param float The measurement.
   * @param weight Optional. The weight by which this measurement is weighted in
   * the aggregation scheme.
   */
  virtual void addMeasurement(const float score, const float weight = 1.f) = 0;

  /**
   * @brief Merge another voxel into this voxel. If the default merging
   * behavior is not desired, the voxel members are exposed to custom
   * manipulation.
   *
   * @param other Voxel to merge into this one.
   * @return True if the merging was successful.
   */
  virtual bool mergeVoxel(const ScoreVoxel& other) = 0;

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

#endif  // PANOPTIC_MAPPING_MAP_SCORES_SCORE_VOXEL_H_
