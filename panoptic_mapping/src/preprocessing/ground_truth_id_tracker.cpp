#include "panoptic_mapping/preprocessing/ground_truth_id_tracker.h"

#include <memory>
#include <unordered_set>
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
  // look for new instances's
  std::unordered_set<int> instances;

  CV_Assert(id_image->depth() == CV_8U);
  const cv::MatIterator_<uchar> begin = id_image->begin<uchar>();
  const cv::MatIterator_<uchar> end = id_image->end<uchar>();
  for (auto it = begin; it != end; ++it) {
    instances.insert(static_cast<int>(*it));
  }

  // allocate new submaps if necessary
  for (const auto& instance : instances) {
    allocateSubmap(instance, submaps);
  }
  printAndResetWarnings();

  // set segmentation image to submap ids
  for (auto it = begin; it != end; ++it) {
    *it = static_cast<uchar>(instance_to_id_[static_cast<int>(*it)]);
  }
}

void GroundTruthIDTracker::processPointcloud(SubmapCollection* submaps,
                                             const Transformation& T_M_C,
                                             const Pointcloud& pointcloud,
                                             const Colors& colors,
                                             std::vector<int>* ids) {
  // iterate through point cloud and replace labels
  for (int& id : *ids) {
    allocateSubmap(id, submaps);
    id = instance_to_id_[id];
  }
  printAndResetWarnings();
}

void GroundTruthIDTracker::allocateSubmap(int instance,
                                          SubmapCollection* submaps) {
  // known existing submap
  if (instance_to_id_.find(instance) != instance_to_id_.end()) {
    return;
  }

  // allocate new submap
  Submap::Config cfg;
  cfg.voxels_per_side = config_.voxels_per_side;

  // check whether the instance code is known
  int new_instance = instance;
  if (!label_handler_->segmentationIdExists(instance)) {
    new_instance = 255;  // code for unknown objects
    auto error_it = unknown_ids.find(instance);
    if (error_it == unknown_ids.end()) {
      unknown_ids[instance] = 1;
    } else {
      unknown_ids[instance] += 1;
    }
  }

  // submap settings
  if (label_handler_->isInstanceClass(new_instance)) {
    cfg.voxel_size = config_.instance_voxel_size;
  } else {
    cfg.voxel_size = config_.background_voxel_size;
  }
  Submap* new_submap = submaps->createSubmap(cfg);
  instance_to_id_[new_instance] = new_submap->getID();
  new_submap->setInstanceID(new_instance);
  new_submap->setClassID(label_handler_->getClassLabel(new_instance));
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
