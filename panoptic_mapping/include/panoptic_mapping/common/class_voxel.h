#ifndef PANOPTIC_MAPPING_COMMON_CLASS_VOXEL_H_
#define PANOPTIC_MAPPING_COMMON_CLASS_VOXEL_H_

#include <vector>

namespace panoptic_mapping {

// Compiler Flag whether to use 16bit or 8bit + averaging strategy.
#define PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING

/**
 * @brief Voxel-type used map classification data.
 */
struct ClassVoxel {
// Define the data type used for counting.
// Options: 16bit unsigned ints (ushort) = [0, 65k].
//          8bit unsigned ints (uint8_t) = [0, 255], with rounding ~ +/- 0.4
//          percent points accurate.
#ifdef PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
  using Counter = uint8_t;
#else
  using Counter = uint16_t;
#endif  // PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING

  // Binary classification.
  Counter belongs_count = 0u;
  Counter foreign_count = 0u;

  // Class-wise classification. Store current_count in the belongs_count member.
  // In the instance case, the index 0 is reserved for the submap the voxel
  // belongs to.
  std::vector<Counter> counts;
  int current_index = -1;
};

// Common class voxel functions.
/**
 * @brief Check whether a given class voxel belongs to its submap.
 *
 * @param voxel Voxel to lookup.
 * @return True if it belongs to the submap.
 */
inline bool classVoxelBelongsToSubmap(const ClassVoxel& voxel) {
  if (voxel.current_index < 0) {
    // This means binary classification is employed.
    return voxel.belongs_count >= voxel.foreign_count;
  } else {
    return voxel.current_index == 0;
  }
}

/**
 * @brief Compute the probability of a voxel belonging to its submap.
 *
 * @param voxel Voxel to lookup.
 * @return Probability in [0, 1].
 */
inline float classVoxelBelongingProbability(const ClassVoxel& voxel) {
  if (voxel.current_index < 0) {
    // This means binary classification is employed.
    return static_cast<float>(voxel.belongs_count) /
           (voxel.foreign_count + voxel.belongs_count);
  } else {
    return static_cast<float>(voxel.counts[0]) /
           std::accumulate(voxel.counts.begin(), voxel.counts.end(),
                           ClassVoxel::Counter(0));
  }
}

/**
 * @brief Increment the binary classification count of a voxel. Existence of the
 * input voxel is not checked.
 *
 * @param voxel Voxel to increment.
 * @param belongs Whether to increment the belonging or foreign count.
 */
inline void classVoxelIncrementBinary(ClassVoxel* voxel, bool belongs) {
  if (belongs) {
    voxel->belongs_count++;
#ifdef PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
    if (voxel->belongs_count == 255u) {
      voxel->belongs_count /= 2u;
      voxel->foreign_count /= 2u;
    }
#endif  // PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
  } else {
    voxel->foreign_count++;
#ifdef PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
    if (voxel->foreign_count == 255u) {
      voxel->belongs_count /= 2u;
      voxel->foreign_count /= 2u;
    }
#endif  // PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
  }
}

/**
 * @brief Increment the class wise classification count of a voxel. TExistence
 * of the input voxel is not checked. Assumes that the counts member of the
 * voxel is already initialized larger than the class index to update.
 *
 * @param voxel Voxel to update.
 * @param class_id Index of the class value to update.
 */
inline void classVoxelIncrementClass(ClassVoxel* voxel, size_t class_id) {
  const ClassVoxel::Counter count = ++(voxel->counts[class_id]);
  if (count >= voxel->belongs_count) {
    voxel->belongs_count = count;
    voxel->current_index = class_id;
  }
#ifdef PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
  if (voxel->belongs_count == 255u) {
    voxel->belongs_count /= 2u;
    for (ClassVoxel::Counter& count : voxel->counts) {
      count /= 2u;
    }
  }
#endif  // PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING
}

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_CLASS_VOXEL_H_
