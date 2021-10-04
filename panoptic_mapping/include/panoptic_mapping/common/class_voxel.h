#ifndef PANOPTIC_MAPPING_COMMON_CLASS_VOXEL_H_
#define PANOPTIC_MAPPING_COMMON_CLASS_VOXEL_H_

#include <climits>
#include <vector>

namespace panoptic_mapping {

// Compiler Flag whether to use 16bit or 8bit + averaging strategy.
#define PANOPTIC_MAPPING_USE_CLASS_VOXELS_AVERAGING

// Compiler flag whether to use 32 bits or 16 bits (reduced)
#define PANOPTIC_MAPPING_USE_REDUCED_UNCERTAINTY_ACCURACY

// How many decimal points of the uncertainty value should be used.
// note value is converted to <int|short> using uncertainty_value * accuracy.
// Thus if uncertainty_value * accuracy > max_size(<int32|short>) an overflow
// occurs.
// Using short as datatype to save memory and having uncertainty values <= 1
// limits accuracy to: 10^-4 using short   and   10^-8 Using int
#define UNCERTAINTY_ACCURACY int(1000)
// Uncertainty is updated as UNCERTAINTY_DECAY_RATE * old_value +
// (1-UNCERTAINTY_DECAY_RATE) * new_value Setting this to 0 will always fully
// update the newest value
#define UNCERTAINTY_DECAY_RATE float(0.5)

#ifdef PANOPTIC_MAPPING_USE_REDUCED_UNCERTAINTY_ACCURACY
using UncertaintyType = short;
#else
using UncertaintyType = int32_t;
#endif  // PANOPTIC_MAPPING_USE_REDUCED_UNCERTAINTY_ACCURACY

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

  // Flag to set if this voxel was labeled manually and contains a groundtruth
  // class
  bool is_gt = false;
};

/**
 * @brief Voxel-type used map classification data.
 */
struct ClassUncertaintyVoxel : ClassVoxel {
  // Uncertainty
  UncertaintyType uncertainty_value = -1;
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
    return voxel.belongs_count > voxel.foreign_count;
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
 * @brief Calculates x * log(x) of the given number. If x is <= returns 0
 * @param x float to calculate entropy for
 * @return  x * log(x) if x > 0 , 0 else
 */
inline float xlogx(float x) {
  if (x <= 0) {
    return 0;
  }
  return x * std::log(x);
}

/**
 * @brief Compute the entropy of the class voting distribution.
 *
 * @param voxel Voxel to lookup.
 * @return Entropy of the distribution
 */
inline float classVoxelEntropy(const ClassVoxel& voxel) {
  float sum = voxel.belongs_count + voxel.foreign_count;

  if (voxel.current_index < 0) {
    return -(xlogx(static_cast<float>(voxel.belongs_count) / sum) +
             xlogx(static_cast<float>(voxel.foreign_count) / sum));
  } else {
    return -std::accumulate(
               voxel.counts.begin(), voxel.counts.end(), 0.0f,
               [sum](float accumulated, ClassVoxel::Counter new_value) {
                 return accumulated +
                        xlogx(static_cast<float>(new_value) / sum);
               }) /
           std::log(voxel.counts.size());
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
 * @brief Updates the uncertainty value of a voxel. Uses UNCERTAINTY_DECAY_RATE
 * * old_value + (1-UNCERTAINTY_DECAY_RATE) * new_value
 *
 * @param voxel Voxel to update.
 * @param uncertainty_value Uncertainty value assigned to this voxel.
 */
inline void classVoxelUpdateUncertainty(ClassUncertaintyVoxel* voxel,
                                        float uncertainty_value) {
  float uncertainty_shifted = uncertainty_value * UNCERTAINTY_ACCURACY;

  // Overflow Check
  const float max_uncertainty_value =
      std::numeric_limits<UncertaintyType>::max();
  if (uncertainty_shifted > max_uncertainty_value) {
    LOG(WARNING) << "Uncertainty value is too big. "
                    "Change uncertainty accuracy or use bigger uncertainty "
                    "type to prevent this.";
    uncertainty_shifted = max_uncertainty_value;
  }
  // Assign values
  if (voxel->uncertainty_value == -1) {
    // Voxel was not yet initialized
    voxel->uncertainty_value = static_cast<UncertaintyType>(
        UNCERTAINTY_DECAY_RATE * uncertainty_shifted);
  } else {
    voxel->uncertainty_value = static_cast<UncertaintyType>(
        UNCERTAINTY_DECAY_RATE * uncertainty_shifted +
        (1 - UNCERTAINTY_DECAY_RATE) * voxel->uncertainty_value);
  }
}

/**
 * Default function if no uncertainty voxels are used, simply return zero
 * @param voxel Voxel for which uncertainty should be returned
 * @return 0.0 Fallback if voxel type does not match
 */
inline float classVoxelUncertainty(const ClassVoxel& voxel) {
  return 0.0;
}

inline float classVoxelUncertainty(const ClassUncertaintyVoxel& voxel) {
  return static_cast<float>(voxel.uncertainty_value) / UNCERTAINTY_ACCURACY;
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
