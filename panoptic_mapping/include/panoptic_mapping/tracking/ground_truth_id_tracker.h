#ifndef PANOPTIC_MAPPING_TRACKING_GROUND_TRUTH_ID_TRACKER_H_
#define PANOPTIC_MAPPING_TRACKING_GROUND_TRUTH_ID_TRACKER_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/label_handler.h"
#include "panoptic_mapping/tracking/id_tracker_base.h"

namespace panoptic_mapping {

/**
 * This id tracker looks up the corresponding ground truth instance id for each
 * submap. Assumes that the input_ids are unique and persistent instance ids.
 */
class GroundTruthIDTracker : public IDTrackerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    Config() { setConfigName("GroundTruthIDTracker"); }

   protected:
    void setupParamsAndPrinting() override;
  };

  GroundTruthIDTracker(const Config& config, std::shared_ptr<Globals> globals,
                       bool print_config = true);
  ~GroundTruthIDTracker() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  /**
   * @brief Check whether this instance id is already allocated or allocate if
   * possible.
   *
   * @param instance Segmentation input id to use.
   * @param submaps Submap collection to allocate the submap in.
   * @param input Input data based upon which the submap will be allocated.
   * @return True if the submap was allocated or tracked.
   */
  bool parseInputInstance(int instance, SubmapCollection* submaps,
                          InputData* input);
  void printAndResetWarnings();

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, GroundTruthIDTracker, std::shared_ptr<Globals>>
      registration_;
  const Config config_;
  std::unordered_map<int, int> instance_to_id_;  // track active maps
  std::unordered_map<int, int> unknown_ids;      // for error handling
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TRACKING_GROUND_TRUTH_ID_TRACKER_H_
