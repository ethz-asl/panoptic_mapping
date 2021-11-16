#ifndef PANOPTIC_MAPPING_TRACKING_PROJECTIVE_ID_TRACKER_H_
#define PANOPTIC_MAPPING_TRACKING_PROJECTIVE_ID_TRACKER_H_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/labels/label_handler_base.h"
#include "panoptic_mapping/map/classification/class_layer.h"
#include "panoptic_mapping/tools/map_renderer.h"
#include "panoptic_mapping/tracking/id_tracker_base.h"
#include "panoptic_mapping/tracking/tracking_info.h"

namespace panoptic_mapping {

/**
 * @brief Uses the input segmentation images and compares them against the
 * rendered map to track associations.
 */
class ProjectiveIDTracker : public IDTrackerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Count iso-surfaces points as valid whose projected depth is within this
    // distance in meters of the measured depth. Negative values indicate
    // multiples of the voxel size.
    float depth_tolerance = -1.0;

    // Which tracking metric to compute. Supported are 'IoU' and 'overlap'.
    std::string tracking_metric = "IoU";

    // Accept matches that have at least this value in the computed trackign
    // metric.
    float match_acceptance_threshold = 0.5;

    // True: Only match submaps and masks that have identical class labels.
    // False: Match any mask to the highest metric submap.
    bool use_class_data_for_matching = true;

    // True: Compute masks by projecting the iso-surface poitns into the frame
    // and account for voxel size. False (experimental): look up each vertex of
    // the depth map in the submap.
    bool use_approximate_rendering = true;

    // Subsample the number of looked up vertices when using
    // 'use_approximate_rendering=false' by this factor squared.
    int rendering_subsampling = 1;

    // Only allocate new submaps for masks that have at least this many pixels.
    int min_allocation_size = 0;

    // Number of threads to use to track submaps in parallel.
    int rendering_threads = std::thread::hardware_concurrency();

    // Renderer settings. The renderer is only used for visualization purposes.
    MapRenderer::Config renderer;

    Config() { setConfigName("ProjectiveIDTracker"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  ProjectiveIDTracker(const Config& config, std::shared_ptr<Globals> globals,
                      bool print_config = true);
  ~ProjectiveIDTracker() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  // Internal methods.
  virtual bool classesMatch(int input_id, int submap_class_id);
  virtual Submap* allocateSubmap(int input_id, SubmapCollection* submaps,
                                 InputData* input);
  TrackingInfoAggregator computeTrackingData(SubmapCollection* submaps,
                                             InputData* input);
  TrackingInfo renderTrackingInfo(const Submap& submap,
                                  const InputData& input) const;

  TrackingInfo renderTrackingInfoApproximate(const Submap& submap,
                                             const InputData& input) const;

  TrackingInfo renderTrackingInfoVertices(const Submap& submap,
                                          const InputData& input) const;

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, ProjectiveIDTracker, std::shared_ptr<Globals>>
      registration_;

  // Members
  const Config config_;

 protected:
  MapRenderer renderer_;  // The renderer is only used if visualization is on.
  cv::Mat rendered_vis_;  // Store visualization data.
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TRACKING_PROJECTIVE_ID_TRACKER_H_
