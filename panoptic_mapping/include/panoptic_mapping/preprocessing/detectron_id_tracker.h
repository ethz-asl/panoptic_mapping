#ifndef PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_ID_TRACKER_H_
#define PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_ID_TRACKER_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/core/camera.h"
#include "panoptic_mapping/preprocessing/id_tracker_base.h"
#include "panoptic_mapping/preprocessing/label_handler.h"
#include "panoptic_mapping/tools/map_renderer.h"

// TEST
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "panoptic_mapping/preprocessing/ground_truth_id_tracker.h"

namespace panoptic_mapping {

/**
 * This id tracker tries to match prdictions of the detectron2 panoptic
 * semgentation (https://github.com/facebookresearch/detectron2) against the
 * map for integration.
 */
// TODO(schmluk): Clean submap creation away from id tracking.
class DetectronIDTracker : public IDTrackerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    float instance_voxel_size = 0.05;
    float background_voxel_size = 0.1;
    float unknown_voxel_size = 0.1;
    float freespace_voxel_size = 0.3;
    int voxels_per_side = 16;

    // TODO(schmluk): clean this up and factor out
    Camera::Config camera;
    MapRenderer::Config renderer;

    // TEST Visualization
    bool paint_by_id = true;         // false: render class label instead
    bool track_against_map = false;  // false: track against last image

    Config() { setConfigName("DetectronIDTracker"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  struct DetectronLabel {
    int id = 0;
    bool is_thing = true;
    int category_id = 0;
    int instance_id = 0;
    float score = 0.f;
  };

  // id - label pairs
  typedef std::unordered_map<int, DetectronLabel> DetectronLabels;

  DetectronIDTracker(const Config& config,
                     std::shared_ptr<LabelHandler> label_handler);
  ~DetectronIDTracker() override = default;

  void processImages(SubmapCollection* submaps, const Transformation& T_M_C,
                     const cv::Mat& depth_image, const cv::Mat& color_image,
                     cv::Mat* id_image) override;

  void processImages(SubmapCollection* submaps, const Transformation& T_M_C,
                     const cv::Mat& depth_image, const cv::Mat& color_image,
                     cv::Mat* id_image, const DetectronLabels& labels);

  void processPointcloud(SubmapCollection* submaps, const Transformation& T_M_C,
                         const Pointcloud& pointcloud, const Colors& colors,
                         std::vector<int>* ids) override;

 private:
  int allocateSubmap(int detectron_id, SubmapCollection* submaps,
                     const DetectronLabels& labels);
  void allocateFreeSpaceSubmap(SubmapCollection* submaps);
  void printAndResetWarnings();

  void trackAgainstPreviousImage(SubmapCollection* submaps,
                                 const Transformation& T_M_C,
                                 const cv::Mat& depth_image,
                                 const cv::Mat& color_image, cv::Mat* id_image,
                                 const DetectronLabels& labels);

  void trackAgainstMap(SubmapCollection* submaps, const Transformation& T_M_C,
                       const cv::Mat& depth_image, const cv::Mat& color_image,
                       cv::Mat* id_image, const DetectronLabels& labels);

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, DetectronIDTracker, std::shared_ptr<LabelHandler>>
      registration_;

  // Members
  const Config config_;
  Camera camera_;
  MapRenderer renderer_;

  std::unordered_map<int, int> unknown_ids;      // for error handling

  // Tracking against previous image information.
  bool is_initialized_ = false;
  Transformation T_M_C_prev_;
  cv::Mat id_image_prev_;

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
