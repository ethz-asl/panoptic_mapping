#include "panoptic_mapping/map/tracked_instance_info.h"

namespace panoptic_mapping {

void CountTrackedInstanceInfo::update(const SegmentInfo& matched_segment_info,
                                      float matching_score) {
  // TODO: the instance score should be weight by the matching and normalized
  score += matched_segment_info.instance_score;

  ClassificationCount new_count =
      ++(class_counts[matched_segment_info.class_id]);
  if (new_count > current_count) {
    current_count = new_count;
    current_index = matched_segment_info.class_id;
  }

  // TODO: for now matching score is not used but should be used for updates
  ++total_count;
}

size_t UncertaintyTrackedInstanceInfo::kNumClasses = 0u;

size_t UncertaintyTrackedInstanceInfo::numClasses() {
    return kNumClasses;
}

void UncertaintyTrackedInstanceInfo::setNumClasses(size_t num_classes) {
  kNumClasses = num_classes;
}

void UncertaintyTrackedInstanceInfo::update(
    const SegmentInfo& matched_segment_info, float matching_score) {}

std::shared_ptr<TrackedInstanceInfo>
TrackedInstanceInfo::make_tracked_instance_info(
    TrackedInstanceInfo::Type type) {
  switch (type) {
    case Type::kCount: {
      return std::make_shared<CountTrackedInstanceInfo>();
    }
    case Type::kUncertainty: {
      return std::make_shared<UncertaintyTrackedInstanceInfo>();
    }
    default: {
      return nullptr;
    }
  }
}

}  // namespace panoptic_mapping
