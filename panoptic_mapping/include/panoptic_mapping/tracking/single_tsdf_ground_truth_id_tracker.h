#ifndef ACTIVE_PANOPTIC_MAPPING_TRACKING_SINGLE_TSDF_GROUND_TRUTH_ID_TRACKER_H_
#define ACTIVE_PANOPTIC_MAPPING_TRACKING_SINGLE_TSDF_GROUND_TRUTH_ID_TRACKER_H_

#include <functional>
#include <memory>
#include <unordered_set>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/labels/label_handler_base.h"
#include "panoptic_mapping/map/classification/class_layer.h"
#include "panoptic_mapping/map/instance_id.h"
#include "panoptic_mapping/tools/map_renderer.h"
#include "panoptic_mapping/tracking/id_tracker_base.h"

namespace panoptic_mapping {

class TrackingInfo;
class TrackingInfoAggregator;
class SegmentInfo;

/**
 * @brief Allocates a single submap to emulate running a monolithic TSDF grid as
 * map representation. Combine this module with the SingleTsdfIntegrator.
 */
class SingleTSDFGroundTruthIDTracker : public IDTrackerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Submap allocation config. Set use class_layer to true to perform label
    // integration.
    Submap::Config submap;

    // Renderer settings. The renderer is only used for visualization purposes.
    MapRenderer::Config renderer;

    Config() { setConfigName("SingleTSDFGroundTruthIDTracker"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  SingleTSDFGroundTruthIDTracker(const Config& config,
                                 std::shared_ptr<Globals> globals);
  ~SingleTSDFGroundTruthIDTracker() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  // Setup utility.
  void setup(SubmapCollection* submaps);

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, SingleTSDFGroundTruthIDTracker, std::shared_ptr<Globals>>
      registration_;
  const Config config_;

  int map_id_;
  bool is_setup_ = false;

  MapRenderer renderer_;  // The renderer is only used if visualization is on.
  cv::Mat rendered_vis_;  // Store visualization data.
};

}  // namespace panoptic_mapping

#endif  // ACTIVE_PANOPTIC_MAPPING_TRACKING_SINGLE_TSDF_TRACKER_H_
