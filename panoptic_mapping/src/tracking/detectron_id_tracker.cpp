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
      ProjectiveIDTracker(config.projective_id_tracker, std::move(globals),
                          false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  addRequiredInput(InputData::InputType::kDetectronLabels);
}

void DetectronIDTracker::processInput(SubmapCollection* submaps,
                                      InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Cache the input labels for submap allocation.
  labels_ = &(input->detectronLabels());

  // Track the predicted (and refined) ids.
  ProjectiveIDTracker::processInput(submaps, input);
}

int DetectronIDTracker::allocateSubmap(int detectron_id,
                                       SubmapCollection* submaps) {
  // Check whether the instance code is known.
  auto it = labels_->find(detectron_id);
  PanopticLabel pan_label;
  if (it == labels_->end()) {
    return -1;
  } else {
    if (!globals_->labelHandler()->segmentationIdExists(
            it->second.category_id)) {
      pan_label = PanopticLabel::kUnknown;
    } else if (it->second.is_thing) {
      pan_label = PanopticLabel::kInstance;
    } else {
      pan_label = PanopticLabel::kBackground;
    }
  }

  // Allocate new submap.
  Submap::Config config = config_.projective_id_tracker.submap_creation;
  switch (pan_label) {
    case PanopticLabel::kInstance: {
      config.voxel_size = config_.projective_id_tracker.instance_voxel_size;
      break;
    }
    case PanopticLabel::kBackground: {
      config.voxel_size = config_.projective_id_tracker.background_voxel_size;
      break;
    }
  }
  config.use_class_layer =
      config_.projective_id_tracker.submap_creation.use_class_layer;
  Submap* new_submap = submaps->createSubmap(config);
  new_submap->setLabel(pan_label);
  const int class_id = it->second.category_id;
  new_submap->setClassID(class_id);
  if (globals_->labelHandler()->segmentationIdExists(class_id)) {
    new_submap->setName(globals_->labelHandler()->getName(class_id));
  }
  return new_submap->getID();
}

bool DetectronIDTracker::classesMatch(int input_id, int submap_class_id) {
  auto it = labels_->find(input_id);
  if (it == labels_->end()) {
    // No known input label.
    return false;
  }
  return it->second.category_id == submap_class_id;
}

}  // namespace panoptic_mapping
