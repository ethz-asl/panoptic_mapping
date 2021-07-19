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
                                           std::shared_ptr<Globals> globals,
                                           bool print_config)
    : config_(config.checkValid()), IDTrackerBase(std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
  addRequiredInputs({InputData::InputType::kSegmentationImage,
                     InputData::InputType::kValidityImage});
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
    parseInputInstance(instance, submaps, input);
  }
  printAndResetWarnings();

  // Set segmentation image to submap ids.
  for (auto it = input->idImage()->begin<int>();
       it != input->idImage()->end<int>(); ++it) {
    *it = instance_to_id_[*it];
  }

  // Allocate free space map if required.
  freespace_allocator_->allocateSubmap(submaps, input);
}

void GroundTruthIDTracker::parseInputInstance(int instance,
                                              SubmapCollection* submaps,
                                              InputData* input) {
  // Known existing submap.
  if (instance_to_id_.find(instance) != instance_to_id_.end()) {
    submaps->getSubmapPtr(instance_to_id_[instance])->setWasTracked(true);
    return;
  }

  // Check whether the instance code is known.
  if (!globals_->labelHandler()->segmentationIdExists(instance)) {
    auto error_it = unknown_ids.find(instance);
    if (error_it == unknown_ids.end()) {
      unknown_ids[instance] = 1;
    } else {
      error_it->second++;
    }
    return;
  }

  // Allocate new submap.
  Submap* new_submap = submap_allocator_->allocateSubmap(
      submaps, input, instance,
      globals_->labelHandler()->getLabelEntry(instance));
  if (new_submap) {
    if (config_.use_ground_truth_instance_ids) {
      new_submap->setInstanceID(instance);
    }
    instance_to_id_[instance] = new_submap->getID();
  } else {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Submap allocation failed for input ID '" << instance << "'.";
  }
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
