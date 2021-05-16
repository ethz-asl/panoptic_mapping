#include "panoptic_mapping/map_management/activity_manager.h"

namespace panoptic_mapping {

void ActivityManager::Config::checkParams() const {
  //  checkParamNE(error_threshold, 0.f, "error_threshold");
}

void ActivityManager::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("required_reobservations", &required_reobservations);
  setupParam("deactivate_after_missed_detections",
             &deactivate_after_missed_detections);
}

ActivityManager::ActivityManager(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void ActivityManager::processSubmaps(SubmapCollection* submaps) {}

}  // namespace panoptic_mapping
