#ifndef PANOPTIC_MAPPING_TRACKING_DETECTRON_ID_TRACKER_H_
#define PANOPTIC_MAPPING_TRACKING_DETECTRON_ID_TRACKER_H_

#include <memory>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/tracking/projective_id_tracker.h"

namespace panoptic_mapping {

/**
 * This id tracker tries to match predictions of the detectron2 panoptic
 * semgentation (https://github.com/facebookresearch/detectron2) against the
 * map for integration.
 */
class DetectronIDTracker : public ProjectiveIDTracker {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    ProjectiveIDTracker::Config projective_id_tracker;

    Config() { setConfigName("DetectronIDTracker"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  DetectronIDTracker(const Config& config, std::shared_ptr<Globals> globals);
  ~DetectronIDTracker() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  int allocateSubmap(int detectron_id, SubmapCollection* submaps) override;
  bool classesMatch(int input_id, int submap_class_id) override;

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, DetectronIDTracker, std::shared_ptr<Globals>>
      registration_;

  // Members.
  const Config config_;

  // Cached labels.
  const DetectronLabels* labels_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TRACKING_DETECTRON_ID_TRACKER_H_
