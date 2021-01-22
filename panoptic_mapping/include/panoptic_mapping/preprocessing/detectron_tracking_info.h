#ifndef PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_TRACKING_INFO_H_
#define PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_TRACKING_INFO_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace panoptic_mapping {

/**
 * Tool to aid tracking metrics computation / comparison.
 */

class TrackingInfoAggregator;

class TrackingInfo {
 public:
  explicit TrackingInfo(int submap_id, int width);
  ~TrackingInfo() = default;

  void insertRenderedPoint(int u, int v, int input_id);

 private:
  friend TrackingInfoAggregator;
  const int submap_id_;
  const int width_;

  // Data.
  std::unordered_set<int> pixel_tracks_;
  std::unordered_map<int, int> counts_;  // <input_id, count>
};

// Summarize the final tracking data.
class TrackingInfoAggregator {
 public:
  TrackingInfoAggregator() = default;
  ~TrackingInfoAggregator() = default;

  // Input.
  void insertTrackingInfos(const std::vector<TrackingInfo>& infos);
  void insertTrackingInfo(const TrackingInfo& info);
  void insertInputImage(const cv::Mat& id_image);

  // Get results. Requires that all input data is already set.
  std::vector<int> getInputIDs() const;
  bool getHighestMetric(int input_id, int* submap_id, float* value,
                        const std::string& metric = "iou");
  bool getAllMetrics(int input_id, std::vector<std::pair<int, float>>* id_value,
                     const std::string& metric = "iou");

 private:
  std::function<float(int, int)> getComputeValueFunction(
      const std::string& metric);
  float computIoU(int input_id, int submap_id) const;
  float computOverlap(int input_id, int submap_id) const;
  // Data.
  std::unordered_map<int, std::unordered_map<int, int>>
      overlap_;  // <input_id, <rendered_id, count>>
  std::unordered_map<int, int> total_rendered_count_;  // <rendered_id, count>
  std::unordered_map<int, int> total_input_count_;     // <input_id, count>
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_TRACKING_INFO_H_
