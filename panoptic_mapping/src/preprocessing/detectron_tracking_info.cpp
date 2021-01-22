#include "panoptic_mapping/preprocessing/detectron_tracking_info.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glog/logging.h>

namespace panoptic_mapping {

// Utility function.
void incrementMap(std::unordered_map<int, int>* map, int id, int value = 1) {
  auto it = map->find(id);
  if (it == map->end()) {
    map->emplace(id, value);
  } else {
    it->second += value;
  }
}

TrackingInfo::TrackingInfo(int submap_id, const Camera::Config& camera)
    : submap_id_(submap_id), width_(camera.width), height_(camera.height) {
  image_ = cv::Mat(cv::Size(width_, height_), CV_32SC1, cv::Scalar(0));
  u_min_ = width_;
  u_max_ = 0;
  v_min_ = height_;
  v_max_ = 0;
}

void TrackingInfo::insertRenderedPoint(int u, int v, int size_x, int size_y) {
  // Mark the left side of the maximum vertex size for later evaluation.
  const int u_min = std::max(0, u - size_x);
  const int width = u + 2 * size_x + 1 - u_min;
  const int v_max = std::min(height_ - 1, v + size_y);
  const int v_min = std::max(0, v - size_y);
  for (int v2 = v_min; v2 <= v_max; ++v2) {
    int& data = image_.at<int>(v2, u_min);
    data = std::max(data, width);
  }
  u_min_ = std::min(u_min_, u_min);
  u_max_ = std::max(u_max_, u_min + width);
  v_min_ = std::min(v_min_, v_min);
  v_max_ = std::max(v_max_, v_max);
}

void TrackingInfo::evaluate(const cv::Mat& id_image) {
  // Pass through the image and lookup which pixels should be covered by the
  // submap. Must be called after all input is inserted.
  for (int v = v_min_; v <= v_max_; ++v) {
    int range = 0;
    for (int u = u_min_; u <= std::min(u_max_, width_ - 1); ++u) {
      range = std::max(range, image_.at<int>(v, u)) - 1;
      if (range > 0) {
        incrementMap(&counts_, id_image.at<int>(v, u));
      }
    }
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
