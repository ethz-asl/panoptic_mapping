#ifndef PANOPTIC_MAPPING_TEST_RANDOMIZATION_UTILS_H_
#define PANOPTIC_MAPPING_TEST_RANDOMIZATION_UTILS_H_
#include <limits>
#include <random>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/binary_count.h"

namespace panoptic_mapping {
namespace test {

// General Randomization tools.
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
  voxel->belongs_count = getRandomInt<ClassificationCount>();
}

}  // namespace test
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TEST_RANDOMIZATION_UTILS_H_
