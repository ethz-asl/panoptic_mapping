#include "panoptic_mapping/tracking/detectron_id_tracker.h"

#include <memory>
#include <unordered_map>
#include <utility>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, DetectronIDTracker,
                                           std::shared_ptr<Globals>>
    DetectronIDTracker::registration_("detectron");

void DetectronIDTracker::Config::checkParams() const {
  checkParamConfig(projective_id_tracker);
}

void DetectronIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("projective_id_tracker", &projective_id_tracker);
}

DetectronIDTracker::DetectronIDTracker(const Config& config,
                                       std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIDTracker(config.projective_id_tracker, std::move(globals)) {
  addRequiredInput(InputData::InputType::kDetectronLabels);
}

void DetectronIDTracker::processInput(SubmapCollection* submaps,
                                      InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Cache the input labels for submap allocation.
  labels_ = &(input->detectronLabels());

  // Track the predicted ids.
  ProjectiveIDTracker::processInput(submaps, input);
}

Submap* DetectronIDTracker::allocateSubmap(int input_id,
                                           SubmapCollection* submaps,
                                           InputData* input) {
  if (input_id == 0) {
    // The id 0 is used for no-predictions in detectron.
    return nullptr;
  }

  // Check whether the instance code is known.
  auto it = labels_->find(input_id);
  if (it == labels_->end()) {
    return nullptr;
  }

  // Parse detectron label.
  LabelEntry label;
  const int class_id = it->second.category_id;
  if (globals_->labelHandler()->segmentationIdExists(class_id)) {
    label = globals_->labelHandler()->getLabelEntry(input_id);
  }
  if (it->second.is_thing) {
    label.label = PanopticLabel::kInstance;
  } else {
    label.label = PanopticLabel::kBackground;
  }

  // Allocate new submap.
  Submap* new_submap =
      submap_allocator_->allocateSubmap(submaps, input, input_id, label);
  new_submap->setClassID(class_id);
  if (globals_->labelHandler()->segmentationIdExists(class_id)) {
    new_submap->setName(globals_->labelHandler()->getName(class_id));
  }
  return new_submap;
}

bool DetectronIDTracker::classesMatch(int input_id, int submap_class_id) {
  if (input_id == 0) {
    // The id 0 is used to denote no-predictions by detectron.
    return false;
  }
  auto it = labels_->find(input_id);
  if (it == labels_->end()) {
    // No known input label.
    return false;
  }
  return it->second.category_id == submap_class_id;
}

}  // namespace panoptic_mapping
