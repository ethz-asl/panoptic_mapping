#include "panoptic_mapping/preprocessing/ground_truth_id_tracker.h"

#include <memory>
#include <utility>
#include <vector>

namespace panoptic_mapping {

GroundTruthIDTracker::Config GroundTruthIDTracker::Config::isValid() const {
  CHECK_GT(voxels_per_side, 0) << "The voxels per side are expected > 0.";
  CHECK_GT(instance_voxel_size, 0.0)
      << "The instance voxel size is expected > 0.0.";
  CHECK_GT(background_voxel_size, 0.0)
      << "The background voxel size is expected > 0.0.";
  return Config(*this);
}

GroundTruthIDTracker::GroundTruthIDTracker(
    const Config& config, std::shared_ptr<LabelHandler> label_handler)
    : config_(config.isValid()), IDTrackerBase(std::move(label_handler)) {}

void GroundTruthIDTracker::processImages(SubmapCollection* submaps,
                                         const Transformation& T_M_C,
                                         const cv::Mat& depth_image,
                                         const cv::Mat& color_image,
                                         cv::Mat* id_image) {
  // iterate over image and replace lables
  for (int v = 0; v < id_image->rows; v++) {
    for (int u = 0; u < id_image->cols; u++) {
      id_image->at<cv::Vec3b>(v, u)[0] =
          readLabelAndAllocateSubmap(id_image->at<cv::Vec3b>(v, u)[0], submaps);
    }
  }
  printAndResetWarnings();
}

void GroundTruthIDTracker::processPointcloud(SubmapCollection* submaps,
                                             const Transformation& T_M_C,
                                             const Pointcloud& pointcloud,
                                             const Colors& colors,
                                             std::vector<int>* ids) {
  // iterate through point cloud and replace labels
  for (auto& id : *ids) {
    id = readLabelAndAllocateSubmap(id, submaps);
  }
  printAndResetWarnings();
}

int GroundTruthIDTracker::readLabelAndAllocateSubmap(
    int instance, SubmapCollection* submaps) {
  auto it = instance_to_id_.find(instance);

  // known existing submap
  if (it != instance_to_id_.end()) {
    return it->second;
  }

  // allocate new submap
  Submap::Config cfg;
  cfg.voxels_per_side = config_.voxels_per_side;

  // check whether the instance code is known
  int id = instance;
  if (!label_handler_->segmentationIdExists(instance)) {
    id = 255;  // code for unknown objects
    auto error_it = unknown_ids.find(instance);
    if (error_it == unknown_ids.end()) {
      unknown_ids[instance] = 1;
    } else {
      unknown_ids[instance] += 1;
    }
  }
  if (label_handler_->isInstanceClass(id)) {
    cfg.voxel_size = config_.instance_voxel_size;
  } else {
    cfg.voxel_size = config_.background_voxel_size;
  }
  submaps->createSubmap(cfg);
}

void GroundTruthIDTracker::printAndResetWarnings() {
  for (auto it : unknown_ids) {
    LOG(WARNING) << "Encountered " << it.second
                 << " occurences of unknown segmentation ID '" << it.first
                 << "'.";
  }
  unknown_ids.clear();
}

}  // namespace panoptic_mapping
