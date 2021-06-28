#include "panoptic_mapping/tracking/single_tsdf_tracker.h"

#include <memory>
#include <unordered_set>
#include <utility>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, SingleTSDFTracker,
                                           std::shared_ptr<Globals>>
    SingleTSDFTracker::registration_("single_tsdf");

void SingleTSDFTracker::Config::checkParams() const {
  checkParamConfig(submap_config);
}

void SingleTSDFTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("submap_config", &submap_config);
  setupParam("use_class_layer", &use_class_layer);
  setupParam("use_detectron", &use_detectron);
}

SingleTSDFTracker::SingleTSDFTracker(const Config& config,
                                     std::shared_ptr<Globals> globals)
    : config_(config.checkValid()), IDTrackerBase(std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  std::unordered_set<InputData::InputType> required_types = {
      InputData::InputType::kColorImage, InputData::InputType::kDepthImage};
  if (config_.use_class_layer_) {
    required_types.insert(InputData::InputType::kSegmentationImage);
  }
  if (config_.use_detectron) {
    required_types.insert(InputData::InputType::kDetectronLabels);
  }

  setRequiredInputs(required_inputs_);
}

void SingleTSDFTracker::processInput(SubmapCollection* submaps,
                                     InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));

  // Check whether the map is already allocated.
  if (!is_setup_) {
    setup(submaps);
  }
}

void SingleTSDFTracker::setup(SubmapCollection* submaps) {
  // Check if there is a loaded map.
  if (submaps->size() > 0) {
    Submap& map = *(submaps->begin());
    if (map.getConfig() != config_.submap_config) {
      LOG(WARNING)
          << "Loaded submap config does not match the specified config.";
    }
    map.setIsActive(true);
    map_id_ = map.getID();
  } else {
    // Allocate the single map.
    Submap* new_submap = submaps->createSubmap(config_.submap_config);
    new_submap->setLabel(PanopticLabel::kBackground);
    map_id_ = new_submap->getID();
  }
  submaps->setActiveFreeSpaceSubmapID(map_id_);
  is_setup_ = true;
}

}  // namespace panoptic_mapping
