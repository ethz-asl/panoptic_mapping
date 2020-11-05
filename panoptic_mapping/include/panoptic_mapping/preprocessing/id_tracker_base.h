#ifndef PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_
#define PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_

#include <memory>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/core/submap_collection.h"
#include "panoptic_mapping/preprocessing/label_handler.h"

namespace panoptic_mapping {

/**
 * This class tracks and matches input to submap ids and allocates new submaps
 * where necessary.
 */
class IDTrackerBase {
 public:
  explicit IDTrackerBase(std::shared_ptr<LabelHandler> label_handler)
      : label_handler_(std::move(label_handler)) {}
  virtual ~IDTrackerBase() = default;

  // interface
  virtual void processImages(SubmapCollection* submaps,
                             const Transformation& T_M_C,
                             const cv::Mat& depth_image,
                             const cv::Mat& color_image, cv::Mat* id_image) {
    LOG(WARNING) << "Using images is not implemented for this IDTracker.";
  }

  virtual void processPointcloud(SubmapCollection* submaps,
                                 const Transformation& T_M_C,
                                 const Pointcloud& pointcloud,
                                 const Colors& colors, std::vector<int>* ids) {
    LOG(WARNING) << "Using point clouds is not implemented for this IDTracker.";
  }

 protected:
  std::shared_ptr<LabelHandler> label_handler_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_
