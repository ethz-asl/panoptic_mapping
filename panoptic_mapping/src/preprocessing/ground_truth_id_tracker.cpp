#include "panoptic_mapping/preprocessing/ground_truth_id_tracker.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

namespace panoptic_mapping {

void GroundTruthIDTracker::Config::checkParams() const {
  checkParamGT(voxels_per_side, 0, "voxels_per_side");
  checkParamGT(instance_voxel_size, 0.f, "instance_voxel_size");
  checkParamGT(background_voxel_size, 0.f, "background_voxel_size");
  checkParamGT(unknown_voxel_size, 0.f, "unknown_voxel_size");
  checkParamGT(freespace_voxel_size, 0.f, "freespace_voxel_size");
}

void GroundTruthIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("instance_voxel_size", &instance_voxel_size);
  setupParam("background_voxel_size", &background_voxel_size);
  setupParam("unknown_voxel_size", &unknown_voxel_size);
  setupParam("freespace_voxel_size", &freespace_voxel_size);
  setupParam("voxels_per_side", &voxels_per_side);
}

GroundTruthIDTracker::GroundTruthIDTracker(
    const Config& config, std::shared_ptr<LabelHandler> label_handler)
    : config_(config.checkValid()), IDTrackerBase(std::move(label_handler)) {}

void GroundTruthIDTracker::processImages(SubmapCollection* submaps,
                                         const Transformation& T_M_C,
                                         const cv::Mat& depth_image,
                                         const cv::Mat& color_image,
                                         cv::Mat* id_image) {
  // Look for new instances.
  std::unordered_set<int> instances;

  CV_Assert(id_image->depth() == CV_8U);
  const cv::MatIterator_<uchar> begin = id_image->begin<uchar>();
  const cv::MatIterator_<uchar> end = id_image->end<uchar>();
  for (auto it = begin; it != end; ++it) {
    instances.insert(static_cast<int>(*it));
  }

  // Allocate new submaps if necessary.
  for (const auto& instance : instances) {
    allocateSubmap(instance, submaps);
  }
  printAndResetWarnings();

  // Set segmentation image to submap ids.
  for (auto it = begin; it != end; ++it) {
    *it = static_cast<uchar>(instance_to_id_[static_cast<int>(*it)]);
  }

  // Allocate free space map if required.
  allocateFreeSpaceSubmap(submaps);
}

void GroundTruthIDTracker::processPointcloud(SubmapCollection* submaps,
                                             const Transformation& T_M_C,
                                             const Pointcloud& pointcloud,
                                             const Colors& colors,
                                             std::vector<int>* ids) {
  // Iterate through point cloud and replace labels.
  for (int& id : *ids) {
    allocateSubmap(id, submaps);
    id = instance_to_id_[id];
  }
  printAndResetWarnings();

  // Allocate free space map if required.
  allocateFreeSpaceSubmap(submaps);
}

void GroundTruthIDTracker::allocateSubmap(int instance,
                                          SubmapCollection* submaps) {
  // known existing submap
  if (instance_to_id_.find(instance) != instance_to_id_.end()) {
    return;
  }

  // check whether the instance code is known
  int new_instance = instance;
  if (!label_handler_->segmentationIdExists(instance)) {
    new_instance = 255;  // reserved code for unknown objects
    auto error_it = unknown_ids.find(instance);
    if (error_it == unknown_ids.end()) {
      unknown_ids[instance] = 1;
    } else {
      unknown_ids[instance] += 1;
    }
  }

  // Allocate new submap.
  Submap::Config cfg;
  cfg.voxels_per_side = config_.voxels_per_side;
  PanopticLabel label = label_handler_->getPanopticLabel(new_instance);
  switch (label) {
    case PanopticLabel::kINSTANCE: {
      cfg.voxel_size = config_.instance_voxel_size;
      break;
    }
    case PanopticLabel::kBACKGROUND: {
      cfg.voxel_size = config_.background_voxel_size;
      break;
    }
    case PanopticLabel::kSPACE: {
      cfg.voxel_size = config_.freespace_voxel_size;
      break;
    }
    case PanopticLabel::kUNKNOWN: {
      cfg.voxel_size = config_.unknown_voxel_size;
      break;
    }
  }
  Submap* new_submap = submaps->createSubmap(cfg);
  instance_to_id_[new_instance] = new_submap->getID();
  new_submap->setInstanceID(new_instance);
  new_submap->setClassID(label_handler_->getClassID(new_instance));
  new_submap->setLabel(label);
}

void GroundTruthIDTracker::allocateFreeSpaceSubmap(SubmapCollection* submaps) {
  if (submaps->getActiveFreeSpaceSubmapID() >= 0) {
    // Currently only allocate one free space submap in the beginning.
    return;
  }

  // Create a new freespace submap.
  Submap::Config config;
  config.voxels_per_side = config_.voxels_per_side;
  config.voxel_size = config_.freespace_voxel_size;
  Submap* space_submap = submaps->createSubmap(config);
  space_submap->setLabel(PanopticLabel::kSPACE);
  space_submap->setInstanceID(-1);  // Will never appear in a seg image.
  submaps->setActiveFreeSpaceSubmapID(space_submap->getID());
}

void GroundTruthIDTracker::printAndResetWarnings() {
  for (auto it : unknown_ids) {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Encountered " << it.second
        << " occurences of unknown segmentation ID '" << it.first << "'.";
  }
  unknown_ids.clear();
}

}  // namespace panoptic_mapping
