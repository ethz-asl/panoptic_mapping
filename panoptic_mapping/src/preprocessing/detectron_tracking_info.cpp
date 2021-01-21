#include "panoptic_mapping/preprocessing/detectron_tracking_info.h"

#include <unordered_map>
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
  if (pixel_tracks_.find(linear_index) != pixel_tracks_.end()) {
    return;
  }
  incrementMap(&counts_, input_id);
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

bool TrackingInfoAggregator::getHighestIoU(int input_id, int* submap_id,
                                           float* iou) {
  CHECK_NOTNULL(submap_id);
  CHECK_NOTNULL(iou);
  if (overlap_.find(input_id) == overlap_.end()) {
    return false;
  }
  int id = 0;
  float best_iou = -1.f;
  for (const auto& id_count_pair : overlap_[input_id]) {
    const float current_iou = computIoU(input_id, id_count_pair.first);
    if (current_iou > best_iou) {
      best_iou = current_iou;
      id = id_count_pair.first;
    }
  }
  if (best_iou >= 0.f) {
    *iou = best_iou;
    *submap_id = id;
    return true;
  }
  return false;
}

bool TrackingInfoAggregator::getAllIoU(int input_id,
                                       std::vector<int>* submap_ids,
                                       std::vector<float>* ious) {
  // Return all overlapping submap ids ordered by IoU.
  CHECK_NOTNULL(submap_ids);
  CHECK_NOTNULL(ious);
}

}  // namespace panoptic_mapping
