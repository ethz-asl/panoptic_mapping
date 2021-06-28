#include "panoptic_mapping/tracking/ground_truth_id_tracker.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, GroundTruthIDTracker,
                                           std::shared_ptr<Globals>>
    GroundTruthIDTracker::registration_("ground_truth");

void GroundTruthIDTracker::Config::checkParams() const {
  checkParamGT(voxels_per_side, 0, "voxels_per_side");
  checkParamGT(instance_voxel_size, 0.f, "instance_voxel_size");
  checkParamGT(background_voxel_size, 0.f, "background_voxel_size");
  checkParamGT(unknown_voxel_size, 0.f, "unknown_voxel_size");
  checkParamGT(freespace_voxel_size, 0.f, "freespace_voxel_size");
  checkParamNE(truncation_distance, 0.f, "truncation_distance");
}

void GroundTruthIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("instance_voxel_size", &instance_voxel_size);
  setupParam("background_voxel_size", &background_voxel_size);
  setupParam("unknown_voxel_size", &unknown_voxel_size);
  setupParam("freespace_voxel_size", &freespace_voxel_size);
  setupParam("voxels_per_side", &voxels_per_side);
  setupParam("use_ground_truth_instance_ids", &use_ground_truth_instance_ids);
  setupParam("truncation_distance", &truncation_distance);
}

GroundTruthIDTracker::GroundTruthIDTracker(const Config& config,
                                           std::shared_ptr<Globals> globals)
    : config_(config.checkValid()), IDTrackerBase(std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void GroundTruthIDTracker::processInput(SubmapCollection* submaps,
                                        InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Look for new instances.
  std::unordered_set<int> instances;
  for (int u = 0; u < input->idImage()->cols; ++u) {
    for (int v = 0; v < input->idImage()->rows; ++v) {
      if (input->validityImage()->at<uchar>(v, u)) {
        instances.insert(input->idImage()->at<int>(v, u));
      }
    }
  }

  // Allocate new submaps if necessary.
  for (const int instance : instances) {
      allocateSubmap(instance, submaps);
  }
  printAndResetWarnings();

  // Set segmentation image to submap ids.
  for (auto it = input->idImage()->begin<int>();
       it != input->idImage()->end<int>(); ++it) {
    *it = instance_to_id_[*it];
  }

  // Allocate free space map if required.
  allocateFreeSpaceSubmap(submaps);
}

void GroundTruthIDTracker::allocateSubmap(int instance,
                                          SubmapCollection* submaps) {
  // Known existing submap.
  if (instance_to_id_.find(instance) != instance_to_id_.end()) {
    submaps->getSubmapPtr(instance_to_id_[instance])->setWasTracked(true);
    return;
  }

  // Check whether the instance code is known.
  int new_instance = instance;
  if (!globals_->labelHandler()->segmentationIdExists(instance)) {
    new_instance = 255;  // reserved code for unknown objects
    auto error_it = unknown_ids.find(instance);
    if (error_it == unknown_ids.end()) {
      unknown_ids[instance] = 1;
    } else {
      unknown_ids[instance] += 1;
    }
  }

  // Allocate new submap.
  Submap::Config config;
  config.voxels_per_side = config_.voxels_per_side;
  PanopticLabel label =
      globals_->labelHandler()->getPanopticLabel(new_instance);
  switch (label) {
    case PanopticLabel::kInstance: {
      config.voxel_size = config_.instance_voxel_size;
      break;
    }
    case PanopticLabel::kBackground: {
      config.voxel_size = config_.background_voxel_size;
      break;
    }
    case PanopticLabel::kFreeSpace: {
      config.voxel_size = config_.freespace_voxel_size;
      break;
    }
    case PanopticLabel::kUnknown: {
      config.voxel_size = config_.unknown_voxel_size;
      break;
    }
  }
  config.truncation_distance = config_.truncation_distance;
  if (config.truncation_distance < 0.f) {
    config.truncation_distance *= -config.voxel_size;
  }
  Submap* new_submap = submaps->createSubmap(config);
  instance_to_id_[new_instance] = new_submap->getID();
  if (config_.use_ground_truth_instance_ids) {
    new_submap->setInstanceID(new_instance);
  }
  new_submap->setClassID(globals_->labelHandler()->getClassID(new_instance));
  new_submap->setLabel(label);
  new_submap->setName(globals_->labelHandler()->getName(new_instance));
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
  config.truncation_distance = config_.truncation_distance;
  if (config.truncation_distance < 0.f) {
    config.truncation_distance *= -config.voxel_size;
  }
  Submap* space_submap = submaps->createSubmap(config);
  space_submap->setLabel(PanopticLabel::kFreeSpace);
  space_submap->setInstanceID(-1);  // Will never appear in a seg image.
  space_submap->setName("FreeSpace");
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
