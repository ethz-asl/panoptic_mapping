#ifndef PANOPTIC_MAPPING_PREPROCESSING_GROUND_TRUTH_ID_TRACKER_H_
#define PANOPTIC_MAPPING_PREPROCESSING_GROUND_TRUTH_ID_TRACKER_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/preprocessing/id_tracker_base.h"
#include "panoptic_mapping/preprocessing/label_handler.h"

namespace panoptic_mapping {

/**
 * This id tracker looks up the corresponding ground truth instance id for each
 * submap.
 */
class GroundTruthIDTracker : public IDTrackerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    bool input_is_mesh_id =
        true;  // If true look up the instance id in the label handler.
    float instance_voxel_size = 0.05;
    float background_voxel_size = 0.1;
    float unknown_voxel_size = 0.1;
    float freespace_voxel_size = 0.3;
    int voxels_per_side = 16;

    Config() { setConfigName("GroundTruthIDTracker"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  GroundTruthIDTracker(const Config& config,
                       std::shared_ptr<LabelHandler> label_handler);
  ~GroundTruthIDTracker() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 private:
  void allocateSubmap(int instance, SubmapCollection* submaps);
  void allocateFreeSpaceSubmap(SubmapCollection* submaps);
  void printAndResetWarnings();

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, GroundTruthIDTracker, std::shared_ptr<LabelHandler>>
      registration_;
  const Config config_;
  std::unordered_map<int, int> instance_to_id_;  // track active maps
  std::unordered_map<int, int> unknown_ids;      // for error handling
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_GROUND_TRUTH_ID_TRACKER_H_
