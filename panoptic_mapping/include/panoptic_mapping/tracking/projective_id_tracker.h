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
#include "panoptic_mapping/common/label_handler.h"
#include "panoptic_mapping/tools/map_renderer.h"
#include "panoptic_mapping/tracking/id_tracker_base.h"
#include "panoptic_mapping/tracking/tracking_info.h"

namespace panoptic_mapping {

/**
 * Uses ground truth segmentation and compares them against the rendered map to
 * track associations.
 */

class ProjectiveIDTracker : public IDTrackerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    // Submap params.
    // TODO(schmluk): Clean submap creation away from id tracking.
    float instance_voxel_size = 0.05;
    float background_voxel_size = 0.1;
    float unknown_voxel_size = 0.1;
    float freespace_voxel_size = 0.3;
    Submap::Config submap_creation;

    // Tracking.
    float depth_tolerance = -1.0;  // m, negative for multiples of voxel size
    std::string tracking_metric = "IoU";  // IoU, overlap
    float match_acceptance_threshold = 0.5;
    bool use_class_data_for_matching = true;

    // Allocation .
    int min_allocation_size = 0;  // #px required to allocate new submap.

    // System params.
    int rendering_threads = std::thread::hardware_concurrency();
    bool input_is_mesh_id = false;  // lookup by instance_id or mesh_id

    // Camera and renderer settings.
    MapRenderer::Config renderer;

    Config() { setConfigName("ProjectiveIDTracker"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  ProjectiveIDTracker(const Config& config, std::shared_ptr<Globals> globals);
  ~ProjectiveIDTracker() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  // Internal methods.
  virtual int allocateSubmap(int instance_id, SubmapCollection* submaps);
  virtual bool classesMatch(int input_id, int submap_class_id);
  void allocateFreeSpaceSubmap(SubmapCollection* submaps);
  TrackingInfoAggregator computeTrackingData(SubmapCollection* submaps,
                                             InputData* input);
  TrackingInfo renderTrackingInfo(const Submap& submap,
                                  const Transformation& T_M_C,
                                  const cv::Mat& depth_image,
                                  const cv::Mat& input_ids) const;

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, ProjectiveIDTracker, std::shared_ptr<Globals>>
      registration_;

  // Members
  const Config config_;

 protected:
  MapRenderer renderer_;  // The renderer is only used if visualization is on.
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TRACKING_PROJECTIVE_ID_TRACKER_H_
