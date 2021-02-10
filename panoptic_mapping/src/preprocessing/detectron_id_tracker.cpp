#include "panoptic_mapping/preprocessing/detectron_id_tracker.h"

#include <memory>
#include <unordered_map>
#include <utility>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, DetectronIDTracker,
                                           std::shared_ptr<LabelHandler>>
    DetectronIDTracker::registration_("detectron");

void DetectronIDTracker::Config::checkParams() const {
  checkParamConfig(projective_id_tracker_config);
}

void DetectronIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("projective_id_tracker_config", &projective_id_tracker_config);
}

DetectronIDTracker::DetectronIDTracker(
    const Config& config, std::shared_ptr<LabelHandler> label_handler)
    : config_(config.checkValid()),
      ProjectiveIDTracker(config_.projective_id_tracker_config,
                          std::move(label_handler)) {
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
    if (it->second.is_thing) {
      pan_label = PanopticLabel::kInstance;
    } else {
      pan_label = PanopticLabel::kBackground;
    }
  }

  // Allocate new submap.
  Submap::Config cfg;
  cfg.voxels_per_side = config_.projective_id_tracker_config.voxels_per_side;
  switch (pan_label) {
    case PanopticLabel::kInstance: {
      cfg.voxel_size = config_.projective_id_tracker_config.instance_voxel_size;
      break;
    }
    case PanopticLabel::kBackground: {
      cfg.voxel_size =
          config_.projective_id_tracker_config.background_voxel_size;
      break;
    }
  }
  Submap* new_submap = submaps->createSubmap(cfg);
  new_submap->setLabel(pan_label);
  new_submap->setClassID(it->second.category_id);
  // TODO(schmluk): add proper data from COCO labels via labelhandler.
  // new_submap->setName(label_handler_->getName(new_instance));
  return new_submap->getID();
}

}  // namespace panoptic_mapping
