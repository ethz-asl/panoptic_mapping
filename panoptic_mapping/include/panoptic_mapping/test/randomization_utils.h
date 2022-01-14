#ifndef PANOPTIC_MAPPING_TEST_RANDOMIZATION_UTILS_H_
#define PANOPTIC_MAPPING_TEST_RANDOMIZATION_UTILS_H_

#include <limits>
#include <random>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/binary_count.h"
#include "panoptic_mapping/map/classification/fixed_count.h"
#include "panoptic_mapping/map/classification/moving_binary_count.h"
#include "panoptic_mapping/map/classification/uncertainty.h"
#include "panoptic_mapping/map/classification/variable_count.h"

namespace panoptic_mapping {
namespace test {

// General Randomization tools.
const size_t kMaxNumClasses = 40u;
static std::default_random_engine random_engine;

template <typename T>
inline T getRandomReal(T min = std::numeric_limits<T>::lowest(),
                       T max = std::numeric_limits<T>::max()) {
  static std::uniform_real_distribution<> random_real_distribution(min, max);
  return random_real_distribution(random_engine);
}

template <typename T>
inline T getRandomInt(T min = std::numeric_limits<T>::lowest(),
                      T max = std::numeric_limits<T>::max()) {
  static std::uniform_int_distribution<> random_int_distribution(min, max);
  return random_int_distribution(random_engine);
}

// Random Initialization.
void randomizeVoxel(TsdfVoxel* voxel) {
  // TSDF Voxel
  voxel->distance = getRandomReal<float>();
  voxel->weight = getRandomReal<float>();
  voxel->color =
      panoptic_mapping::Color(getRandomInt<uint8_t>(), getRandomInt<uint8_t>(),
                              getRandomInt<uint8_t>());
}

void randomizeVoxel(BinaryCountVoxel* voxel) {
  voxel->belongs_count = getRandomInt<ClassificationCount>();
  voxel->foreign_count = getRandomInt<ClassificationCount>();
}

void randomizeVoxel(FixedCountVoxel* voxel) {
  const size_t num_classes = getRandomInt(size_t(0), kMaxNumClasses);
  voxel->counts.resize(num_classes);
  voxel->current_count = 0;
  voxel->current_index = -1;
  voxel->total_count = 0;
  for (size_t i = 0; i < num_classes; ++i) {
    const ClassificationCount count = getRandomInt<ClassificationCount>();
    voxel->counts[i] = count;
    voxel->total_count += count;
    if (count > voxel->current_count) {
      voxel->current_count = count;
      voxel->current_index = i;
    }
  }
}

void randomizeVoxel(MovingBinaryCountVoxel* voxel) {
  voxel->belongs_count = getRandomInt<uint8_t>();
  voxel->foreign_count = getRandomInt<uint8_t>();
}

void randomizeVoxel(VariableCountVoxel* voxel) {
  voxel->counts.clear();
  for (size_t i = 0; i < getRandomInt<size_t>(0, kMaxNumClasses); ++i) {
    voxel->counts[getRandomInt<int16_t>()] =
        getRandomInt<ClassificationCount>();
  }
  voxel->total_count = 0;
  voxel->current_count = 0;
  voxel->current_index = -1;
  for (const auto& id_count_pair : voxel->counts) {
    voxel->total_count += id_count_pair.second;
    if (id_count_pair.second > voxel->current_count) {
      voxel->current_count = id_count_pair.second;
      voxel->current_index = id_count_pair.first;
    }
  }
}

void randomizeVoxel(UncertaintyVoxel* voxel) {
  randomizeVoxel(static_cast<FixedCountVoxel*>(voxel));
  voxel->is_ground_truth = getRandomInt(0, 1);
  voxel->uncertainty = getRandomReal<float>();
}

}  // namespace test
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TEST_RANDOMIZATION_UTILS_H_
