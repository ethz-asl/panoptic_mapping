#include "panoptic_mapping/preprocessing/detectron_tracking_info.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glog/logging.h>

namespace panoptic_mapping {

TrackingInfo::TrackingInfo(int submap_id, int width)
    : submap_id_(submap_id), width_(width) {}

// Utility function.
void incrementMap(std::unordered_map<int, int>* map, int id, int value = 1) {
  auto it = map->find(id);
  if (it == map->end()) {
    map->emplace(id, value);
  } else {
    it->second += value;
  }
}

void TrackingInfo::insertRenderedPoint(int u, int v, int input_id) {
  // Avoid duplicates here.
  const int linear_index = u + width_ * v;
  if (pixel_tracks_.find(linear_index) == pixel_tracks_.end()) {
    incrementMap(&counts_, input_id);
    pixel_tracks_.insert(linear_index);
  }
}

void TrackingInfoAggregator::insertTrackingInfos(
    const std::vector<TrackingInfo>& infos) {
  for (const TrackingInfo& info : infos) {
    insertTrackingInfo(info);
  }
}

void TrackingInfoAggregator::insertTrackingInfo(const TrackingInfo& info) {
  // Parse all overlaps. We don't check for duplicate data here since by
  // construction each submap should be parsed separately and only once.
  for (const auto& id_count_pair : info.counts_) {
    incrementMap(&total_rendered_count_, info.submap_id_, id_count_pair.second);
    auto it = overlap_.find(id_count_pair.first);
    if (it == overlap_.end()) {
      it = overlap_
               .insert(std::pair(id_count_pair.first,
                                 std::unordered_map<int, int>()))
               .first;
    }
    it->second[info.submap_id_] = id_count_pair.second;
  }
}

void TrackingInfoAggregator::insertInputImage(const cv::Mat& id_image) {
  for (int u = 0; u < id_image.cols; ++u) {
    for (int v = 0; v < id_image.rows; ++v) {
      incrementMap(&total_input_count_, id_image.at<int>(v, u));
    }
  }
}

std::vector<int> TrackingInfoAggregator::getInputIDs() const {
  // Get a vector containing all unique input ids.
  std::vector<int> result;
  result.reserve(total_input_count_.size());
  for (const auto& id_count : total_input_count_) {
    result.emplace_back(id_count.first);
  }
  return result;
}

float TrackingInfoAggregator::computIoU(int input_id, int submap_id) const {
  // This assumes that input and submap id exist.
  const int count = overlap_.at(input_id).at(submap_id);
  return static_cast<float>(count) /
         (static_cast<float>(total_rendered_count_.at(submap_id) +
                             total_input_count_.at(input_id) - count));
}

float TrackingInfoAggregator::computOverlap(int input_id, int submap_id) const {
  // This assumes that input and submap id exist.
  return static_cast<float>(overlap_.at(input_id).at(submap_id)) /
         static_cast<float>(total_input_count_.at(input_id));
}

std::function<float(int, int)> TrackingInfoAggregator::getComputeValueFunction(
    const std::string& metric) {
  if (metric == "overlap") {
    return [=](int input_id, int submap_id) {
      return this->computOverlap(input_id, submap_id);
    };
  }
  if (metric != "iou" && metric != "IoU") {
    LOG(WARNING) << "Unknown tracking metric '" << metric
                 << "', using 'IoU' instead.";
  }
  return [=](int input_id, int submap_id) {
    return this->computIoU(input_id, submap_id);
  };
}

bool TrackingInfoAggregator::getHighestMetric(int input_id, int* submap_id,
                                              float* value,
                                              const std::string& metric) {
  CHECK_NOTNULL(submap_id);
  CHECK_NOTNULL(value);
  if (overlap_.find(input_id) == overlap_.end()) {
    return false;
  }

  auto value_function = getComputeValueFunction(metric);
  int id = 0;
  float best_value = -1.f;
  for (const auto& id_count_pair : overlap_[input_id]) {
    const float current_value = value_function(input_id, id_count_pair.first);
    if (current_value > best_value) {
      best_value = current_value;
      id = id_count_pair.first;
    }
  }
  if (best_value >= 0.f) {
    *value = best_value;
    *submap_id = id;
    return true;
  }
  return false;
}

bool TrackingInfoAggregator::getAllMetrics(
    int input_id, std::vector<std::pair<int, float>>* id_value,
    const std::string& metric) {
  CHECK_NOTNULL(id_value);
  // Return all overlapping submap ids ordered by IoU.
  if (overlap_.find(input_id) == overlap_.end()) {
    return false;
  }
  auto value_function = getComputeValueFunction(metric);
  id_value->clear();
  id_value->reserve(overlap_[input_id].size());
  for (const auto& id_count_pair : overlap_[input_id]) {
    id_value->emplace_back(id_count_pair.first,
                           value_function(input_id, id_count_pair.first));
  }
  if (id_value->empty()) {
    return false;
  }
  std::sort(std::begin(*id_value), std::end(*id_value),
            [&](const auto& lhs, const auto& rhs) {
              return lhs.second > rhs.second;
            });
  return true;
}

}  // namespace panoptic_mapping
