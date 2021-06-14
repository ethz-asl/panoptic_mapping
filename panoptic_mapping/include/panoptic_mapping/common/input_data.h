#ifndef PANOPTIC_MAPPING_COMMON_INPUT_DATA_H_
#define PANOPTIC_MAPPING_COMMON_INPUT_DATA_H_

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {
class InputDataUser;

/**
 * All custom types of input data are defined here.
 */

// Labels supplied by the detectron2 network.
struct DetectronLabel {
  int id = 0;
  bool is_thing = true;
  int category_id = 0;
  int instance_id = 0;
  float score = 0.f;
};
typedef std::unordered_map<int, DetectronLabel> DetectronLabels;  // <id-label>

/**
 * A class that wraps all input data to be processed into a common structure.
 * Optional fields are also included here but the data is only set if required.
 */
class InputData {
 public:
  // Lists all types of possible inputs.
  enum class InputType {
    kDepthImage,
    kColorImage,
    kSegmentationImage,
    kDetectronLabels,
    kVertexMap
  };
  static std::string inputTypeToString(InputType type) {
    switch (type) {
      case InputType::kDepthImage:
        return "Depth Image";
      case InputType::kColorImage:
        return "Color Image";
      case InputType::kSegmentationImage:
        return "Segmentation Image";
      case InputType::kDetectronLabels:
        return "Detectron Labels";
      case InputType::kVertexMap:
        return "Vertex Map";
    }
  }

  InputData() = default;
  virtual ~InputData() = default;

  // Input.
  void setT_M_C(const Transformation& T_M_C) { T_M_C_ = T_M_C; }
  void setTimeStamp(double timestamp) { timestamp_ = timestamp; }
  void setDepthImage(const cv::Mat& depth_image) {
    depth_image_ = depth_image;
    contained_inputs_.insert(InputType::kDepthImage);
  }
  void setColorImage(const cv::Mat& color_image) {
    color_image_ = color_image;
    contained_inputs_.insert(InputType::kColorImage);
  }
  void setIdImage(const cv::Mat& id_image) {
    id_image_ = id_image;
    contained_inputs_.insert(InputType::kSegmentationImage);
  }
  void setDetectronLabels(const DetectronLabels& labels) {
    detectron_labels_ = labels;
    contained_inputs_.insert(InputType::kDetectronLabels);
  }
  void setVertexMap(const cv::Mat& vertex_map) {
    vertex_map_ = vertex_map;
    contained_inputs_.insert(InputType::kVertexMap);
  }

  // Access.
  const Transformation& T_M_C() const { return T_M_C_; }
  double timestamp() const { return timestamp_; }
  const cv::Mat& depthImage() const { return depth_image_; }
  const cv::Mat& colorImage() const { return color_image_; }
  cv::Mat* idImage() { return &id_image_; }
  const DetectronLabels& detectronLabels() const { return detectron_labels_; }
  const cv::Mat& vertexMap() const { return vertex_map_; }

 private:
  friend InputDataUser;

  // Permanent data.
  Transformation T_M_C_;    // Transform Camera (sensor) to Mission.
  double timestamp_ = 0.0;  // Timestamp of the inputs.

  // Common data.
  cv::Mat depth_image_;  // Float depth image (CV_32FC1).
  cv::Mat color_image_;  // BGR (CV_8U).
  cv::Mat id_image_;     // Mutable assigned ids as ints (CV_32SC1).
  cv::Mat vertex_map_;   // XYZ points (CV32FC3), can be compute via camera.

  // Optional data.
  DetectronLabels detectron_labels_;

  // Content tracking.
  std::unordered_set<InputType> contained_inputs_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_INPUT_DATA_H_
