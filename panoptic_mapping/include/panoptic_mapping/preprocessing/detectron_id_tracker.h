#ifndef PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_ID_TRACKER_H_
#define PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_ID_TRACKER_H_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/preprocessing/id_tracker_base.h"
#include "panoptic_mapping/preprocessing/label_handler.h"
#include "panoptic_mapping/preprocessing/tracking_info.h"
#include "panoptic_mapping/tools/map_renderer.h"

// TEST
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "panoptic_mapping/preprocessing/ground_truth_id_tracker.h"

namespace panoptic_mapping {

// TODO(schmluk): This does not run atm. Replace this tracker with the
// projective
// one and refactor the label readout.

/**
 * This id tracker tries to match prdictions of the detectron2 panoptic
 * semgentation (https://github.com/facebookresearch/detectron2) against the
 * map for integration.
 */
class DetectronIDTracker : public IDTrackerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    // Allocation TODO(schmluk): Clean submap creation away from id tracking.
    float instance_voxel_size = 0.05;
    float background_voxel_size = 0.1;
    float unknown_voxel_size = 0.1;
    float freespace_voxel_size = 0.3;
    int voxels_per_side = 16;

    // Tracking
    float depth_tolerance = -1.0;  // m, negative for multiples of voxel size
    std::string tracking_metric = "IoU";  // iou, overlap
    float match_acceptance_threshold = 0.5;

    // Camera and renderer settings.
    std::string camera_namespace = "";
    Camera::Config camera;
    MapRenderer::Config renderer;

    // System params.
    int rendering_threads = std::thread::hardware_concurrency();

    // TEST Visualization
    GroundTruthIDTracker::Config gt_id_tracker_config;
    bool debug = false;

    Config() { setConfigName("DetectronIDTracker"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  DetectronIDTracker(const Config& config,
                     std::shared_ptr<LabelHandler> label_handler);
  ~DetectronIDTracker() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 private:
  int allocateSubmap(int detectron_id, SubmapCollection* submaps,
                     const DetectronLabels& labels);
  void allocateFreeSpaceSubmap(SubmapCollection* submaps);

  TrackingInfo renderTrackingInfo(const Submap& submap,
                                  const Transformation& T_M_C,
                                  const cv::Mat& depth_image,
                                  const cv::Mat& input_ids) const;

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, DetectronIDTracker, std::shared_ptr<LabelHandler>>
      registration_;

  // Members
  const Config config_;
  Camera camera_;
  MapRenderer renderer_;

  // TEST
  ros::NodeHandle nh_;
  ros::Publisher input_pub_;
  ros::Publisher color_pub_;
  ros::Publisher rendered_pub_;
  ros::Publisher tracking_pub_;
  std::unordered_map<int, int> instance_to_id_;  // track active maps
  GroundTruthIDTracker gt_tracker_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_ID_TRACKER_H_
