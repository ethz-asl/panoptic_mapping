#ifndef PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_
#define PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/input_data_user.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/preprocessing/label_handler.h"

namespace panoptic_mapping {

/**
 * This class tracks and matches input to submap ids and allocates new submaps
 * where necessary.
 */
class IDTrackerBase : public InputDataUser {
 public:
  explicit IDTrackerBase(std::shared_ptr<LabelHandler> label_handler)
      : label_handler_(std::move(label_handler)) {
    // Per default require all three images.
    addRequiredInput(InputData::InputType::kDepthImage);
    addRequiredInput(InputData::InputType::kColorImage);
    addRequiredInput(InputData::InputType::kSegmentationImage);
  }
  ~IDTrackerBase() override = default;

  // Interface;
  virtual void processInput(SubmapCollection* submaps, InputData* input) = 0;

  // Visualization callbacks.
  void setVisualizationCallback(
      std::function<void(const cv::Mat&, const std::string&)> callback) {
    visualization_callback_ = std::move(callback);
    visualize_ = true;
  }

 protected:
  std::shared_ptr<LabelHandler> label_handler_;

  // Visualization
  bool visualizationIsOn() const { return visualize_; }
  void visualize(const cv::Mat& image, const std::string& name) {
    if (visualize_) {
      visualization_callback_(image, name);
    }
  }

 private:
  bool visualize_ = false;
  std::function<void(const cv::Mat&, const std::string&)>
      visualization_callback_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_
