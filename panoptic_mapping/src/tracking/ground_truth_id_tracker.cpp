#include "panoptic_mapping/tracking/ground_truth_id_tracker.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, GroundTruthIDTracker,
                                           std::shared_ptr<Globals>>
    GroundTruthIDTracker::registration_("ground_truth");

void GroundTruthIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
}

GroundTruthIDTracker::GroundTruthIDTracker(const Config& config,
                                           std::shared_ptr<Globals> globals)
    : config_(config.checkValid()), IDTrackerBase(std::move(globals)) {

  addRequiredInputs({InputData::InputType::kSegmentationImage,
                     InputData::InputType::kValidityImage});
}

void GroundTruthIDTracker::processInput(SubmapCollection* submaps,
                                        InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Look for new instances that are within integration range.
  std::unordered_set<int> instances;
  for (int u = 0; u < input->idImage().cols; ++u) {
    for (int v = 0; v < input->idImage().rows; ++v) {
      if (input->validityImage().at<uchar>(v, u)) {
        instances.insert(input->idImage().at<int>(v, u));
      }
    }
  }

  // Allocate new submaps if necessary.
  for (const int instance : instances) {
    parseInputInstance(instance, submaps, input);
  }
  printAndResetWarnings();

  // Set segmentation image to submap ids.
  for (auto it = input->idImagePtr()->begin<int>();
       it != input->idImagePtr()->end<int>(); ++it) {
    auto it2 = instance_to_id_.find(*it);
    if (it2 == instance_to_id_.end()) {
      *it = -1;
    } else {
      *it = it2->second;
    }
  }

  // Allocate free space map if required.
  freespace_allocator_->allocateSubmap(submaps, input);
}

bool GroundTruthIDTracker::parseInputInstance(int instance,
                                              SubmapCollection* submaps,
                                              InputData* input) {
  // Known existing submap.
  auto it = instance_to_id_.find(instance);
  if (it != instance_to_id_.end()) {
    if (submaps->submapIdExists(it->second)) {
      submaps->getSubmapPtr(it->second)->setWasTracked(true);
      return true;
    } else {
      LOG_IF(WARNING, config_.verbosity >= 2)
          << "Submap '" << it->second << "' for instance ID '" << instance
          << "' has been deleted.";
      instance_to_id_.erase(it);
    }
  }

  // Check whether the instance code is known.
  if (!globals_->labelHandler()->segmentationIdExists(instance)) {
    auto error_it = unknown_ids.find(instance);
    if (error_it == unknown_ids.end()) {
      unknown_ids[instance] = 1;
    } else {
      error_it->second++;
    }
    return false;
  }

  // Allocate new submap.
  Submap* new_submap = submap_allocator_->allocateSubmap(
      submaps, input, instance,
      globals_->labelHandler()->getLabelEntry(instance));
  if (new_submap) {
    new_submap->setInstanceID(instance);
    instance_to_id_[instance] = new_submap->getID();
    return true;
  } else {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Submap allocation failed for input ID '" << instance << "'.";
    return false;
  }
}

void GroundTruthIDTracker::printAndResetWarnings() {
  if (config_.verbosity < 2) {
    unknown_ids.clear();
    return;
  }
  for (auto it : unknown_ids) {
    LOG(WARNING) << "Encountered " << it.second
                 << " occurences of unknown segmentation ID '" << it.first
                 << "'.";
  }
  unknown_ids.clear();
}

}  // namespace panoptic_mapping
