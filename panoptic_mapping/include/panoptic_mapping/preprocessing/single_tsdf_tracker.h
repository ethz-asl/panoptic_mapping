#ifndef PANOPTIC_MAPPING_PREPROCESSING_SINGLE_TSDF_TRACKER_H_
#define PANOPTIC_MAPPING_PREPROCESSING_SINGLE_TSDF_TRACKER_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/preprocessing/id_tracker_base.h"
#include "panoptic_mapping/preprocessing/label_handler.h"

namespace panoptic_mapping {

/**
 * Allocates a single submap to emulate running a single TSDF grid as map
 * representation.
 */
class SingleTSDFTracker : public IDTrackerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    float voxel_size = 0.1;
    float truncation_distance = 0.0;  // Defaults to 2x voxel size.
    int voxels_per_side = 16;

    Config() { setConfigName("SingleTSDFTracker"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  SingleTSDFTracker(const Config& config, std::shared_ptr<Globals> globals);
  ~SingleTSDFTracker() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, SingleTSDFTracker, std::shared_ptr<Globals>>
      registration_;
  const Config config_;

  int map_id_;
  bool is_setup_ = false;

  // Setup utility.
  void setup(SubmapCollection* submaps);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_SINGLE_TSDF_TRACKER_H_
