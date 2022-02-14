#include "panoptic_mapping/map/tracked_instance_info.h"

namespace panoptic_mapping {

void CountTrackedInstanceInfo::update(const SegmentInfo& matched_segment_info,
                                      float matching_score) {
  // TODO: the instance score should be weighted by the matching and normalized
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

void UncertaintyTrackedInstanceInfo::update(
    const SegmentInfo& matched_segment_info, float matching_score) {
  double numerator_term =
      matched_segment_info.class_score * matched_segment_info.instance_score;
  double denominator_term = matched_segment_info.instance_score;

  auto element_iter = class_probs.find(matched_segment_info.class_id);
  if (element_iter == class_probs.end()) {
    class_probs.insert(std::pair<int, std::pair<double, double>>(
        matched_segment_info.class_id, {numerator_term, denominator_term}));
    return;
  }

  auto& prob_num_den = element_iter->second;
  prob_num_den.first += numerator_term;
  prob_num_den.second += denominator_term;

  float prob = static_cast<float>(prob_num_den.first / prob_num_den.second);
  if (prob > current_max_prob) {
    current_index = matched_segment_info.class_id;
    current_max_prob = prob;
  }
}

size_t BayesianTrackedInstanceInfo::kNumClasses = 0u;

void BayesianTrackedInstanceInfo::setNumClasses(size_t num_classes) {
  // Include also the void class
  kNumClasses = num_classes;
}

void BayesianTrackedInstanceInfo::update(
    const SegmentInfo& matched_segment_info, float matching_score) {
  LOG_IF(FATAL, matched_segment_info.class_probs.size() != class_probs.size())
      << "The shape of the class distribution does not match!";

  for (size_t class_id = 0; class_id < class_probs.size(); ++class_id) {
    class_probs[class_id] *= matched_segment_info.class_probs[class_id];
  }

  // Compute normalization factor
  float normalization_factor =
      std::accumulate(class_probs.begin(), class_probs.end(), 0.f);

  // Normalize probabilities
  std::transform(class_probs.begin(), class_probs.end(), class_probs.begin(),
                 [&normalization_factor](float f) -> float {
                   return f / normalization_factor;
                 });

  // Update current index and its probability
  auto max_prob_it = std::max_element(class_probs.begin(), class_probs.end());
  current_index =
      static_cast<int>(std::distance(class_probs.begin(), max_prob_it));
  current_index_prob = *max_prob_it;
}

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
    case Type::kBayesian: {
      return std::make_shared<BayesianTrackedInstanceInfo>();
    }
    default: {
      LOG(FATAL) << "Unrecognized TrackeInstanceInfo::Type!" << std::endl;
    }
  }
}

}  // namespace panoptic_mapping
