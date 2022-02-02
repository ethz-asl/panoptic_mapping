#include <unordered_map>

#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {
  
struct TrackedInstanceInfo {
  double score = 0.f;
  int current_index = 0;
  ClassificationCount total_count = 0;
  ClassificationCount current_count = 0;
  std::unordered_map<int, ClassificationCount> class_counts;

  void update(float instance_score, int class_id, int matching_score) {
    // TODO: the instance score should be weight by the matching and normalized
    score += instance_score;

    ClassificationCount new_count = ++(class_counts[class_id]);
    if (new_count > current_count) {
      current_count = new_count;
      current_index = class_id;
    }

    // TODO: for now matching score is not used but should be used for updates
    ++total_count;
  }
};

}  // namespace panoptic_mapping
