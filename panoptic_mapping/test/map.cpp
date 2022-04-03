#include <gtest/gtest.h>

#include "panoptic_mapping/map/classification/variable_bayesian.h"

using namespace panoptic_mapping;

TEST(Classification, VariableBayesian) {
  // Series of observations
  const std::vector<std::pair<int, float>> observations = {
      {4, 0.7f}, {4, 0.9f}, {4, 0.8f}, {3, 0.7f}, 
      {3, 0.8f}, {3, 0.7f}, {3, 0.7f}, {3, 0.9f}};

  // Instantiate variable bayesian classification voxel;
  VariableBayesianVoxel voxel;

  for (auto const& [id, score] : observations) {
    voxel.incrementCount(id, score);
  }

  EXPECT_EQ(voxel.getBelongingID(), 3);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
