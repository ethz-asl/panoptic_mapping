#include "panoptic_mapping/map_management/activity_manager.h"

#include <unordered_set>

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
    : config_(config.checkValid()) {}

void ActivityManager::processSubmaps(SubmapCollection* submaps) {
  CHECK_NOTNULL(submaps);
  std::unordered_set<int> submaps_to_delete;
  for (Submap& submap : *submaps) {
    // Parse only active object maps.
    // NOTE(schmluk): Could be extended to free space for global consistency.
    if (!submap.isActive() || submap.getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }

    // Check for re-detections of new submaps.
    if (!checkRequiredRedetection(&submap)) {
      submaps_to_delete.insert(submap.getID());
      continue;
    }

    // Check tracking for active submaps.
    checkMissedDetections(&submap);
  }

  // Remove requested submaps.
  for (const int id : submaps_to_delete) {
    submaps->removeSubmap(id);
  }

  // Reset.
  for (Submap& submap : *submaps) {
    submap.setWasTracked(false);
  }
}

bool ActivityManager::checkRequiredRedetection(Submap* submap) {
  // Check the submap was re-detected in X consecutive frames after allocation.
  if (config_.required_reobservations <= 0) {
    return true;
  }
  const int submap_id = submap->getID();
  auto it = submap_redetection_counts_.find(submap_id);
  if (it == submap_redetection_counts_.end()) {
    // This is a new submap.
    submap_redetection_counts_[submap_id] = config_.required_reobservations;
    return true;
  }
  if (it->second <= 0) {
    // This submap already passed the re-detection test.
    return true;
  }
  if (submap->wasTracked()) {
    // Was re-observed, decrease remaining required re-observations.
    it->second--;
    return true;
  }
  // Not detected, remove the submap.
  return false;
}

void ActivityManager::checkMissedDetections(Submap* submap) {
  // Check whether a submap was not detected for X consecutive frames.
  if (config_.deactivate_after_missed_detections <= 0) {
    return;
  }
  if (submap->wasTracked()) {
    // Was tracked so reset the counter.
    submap_missed_detection_counts_.erase(submap->getID());
  } else {
    auto it = submap_missed_detection_counts_.find(submap->getID());
    if (it == submap_missed_detection_counts_.end()) {
      // First missed detection, add to counter.
      it = submap_missed_detection_counts_.insert(
          submap_missed_detection_counts_.end(),
          std::pair(submap->getID(),
                    config_.deactivate_after_missed_detections));
    }
    it->second--;
    if (it->second <= 0) {
      submap->finishActivePeriod();
    }
  }
}

}  // namespace panoptic_mapping
