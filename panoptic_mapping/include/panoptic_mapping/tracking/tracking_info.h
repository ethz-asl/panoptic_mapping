#ifndef PANOPTIC_MAPPING_TRACKING_TRACKING_INFO_H_
#define PANOPTIC_MAPPING_TRACKING_TRACKING_INFO_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/common/camera.h"

namespace panoptic_mapping {

/**
 * Tool to aid tracking metrics computation / comparison.
 */

// NOTE(schmluk): Currently performs input validity checks (max ranges etc)
// independent of other modules, maybe best to do this once and combine.

class TrackingInfoAggregator;

class TrackingInfo {
 public:
  explicit TrackingInfo(int submap_id) : submap_id_(submap_id) {}
  TrackingInfo(int submap_id, Camera::Config camera);
  ~TrackingInfo() = default;

  // Approximate rendering.
  void insertRenderedPoint(int u, int v, int size_x, int size_y);
  void evaluate(const cv::Mat& id_image, const cv::Mat& depth_image);

  // Vertex rendering.
  void insertVertexPoint(int input_id);
  void insertVertexVisualizationPoint(int u, int v) {
    points_.push_back({u, v});
  };
  const std::vector<Eigen::Vector2i>& getPoints() const { return points_; }
  int getSubmapID() const { return submap_id_; }

 private:
  friend TrackingInfoAggregator;
  const int submap_id_;
  std::unordered_map<int, int> counts_;  // <input_id, count>

  // Approximate rendering.
  const Camera::Config camera_;
  int u_min_, u_max_, v_min_, v_max_;
  cv::Mat image_;

  // Visualization data vertex rendering.
  std::vector<Eigen::Vector2i> points_;
};

// Summarize the final tracking data.
class TrackingInfoAggregator {
 public:
  TrackingInfoAggregator() = default;
  ~TrackingInfoAggregator() = default;

  // Input.
  void insertTrackingInfos(const std::vector<TrackingInfo>& infos);
  void insertTrackingInfo(const TrackingInfo& info);
  void insertInputImage(const cv::Mat& id_image, const cv::Mat& depth_image,
                        const Camera::Config& camera,
                        int rendering_subsampling);

  // Get results. Requires that all input data is already set.
  std::vector<int> getInputIDs() const;
  bool getHighestMetric(int input_id, int* submap_id, float* value,
                        const std::string& metric = "iou") const;
  bool getAllMetrics(int input_id, std::vector<std::pair<int, float>>* id_value,
                     const std::string& metric = "iou") const;

  // Get data.
  int getNumberOfInputPixels(int input_id) const;
  int getNumberOfSubmapPixels(int submap_id) const;
  int getNumberOfOverlappingPixels(int input_id, int submap_id) const;

 private:
  std::function<float(int, int)> getComputeValueFunction(
      const std::string& metric) const;
  float computIoU(int input_id, int submap_id) const;
  float computOverlap(int input_id, int submap_id) const;

  // Data.
  std::unordered_map<int, std::unordered_map<int, int>>
      overlap_;  // <input_id, <rendered_id, count>>
  std::unordered_map<int, int> total_rendered_count_;  // <rendered_id, count>
  std::unordered_map<int, int> total_input_count_;     // <input_id, count>
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TRACKING_TRACKING_INFO_H_
