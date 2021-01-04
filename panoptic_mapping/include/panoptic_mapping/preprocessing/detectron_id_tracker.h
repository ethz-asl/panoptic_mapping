#ifndef PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_ID_TRACKER_H_
#define PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_ID_TRACKER_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/preprocessing/id_tracker_base.h"
#include "panoptic_mapping/preprocessing/label_handler.h"

namespace panoptic_mapping {

/**
 * This id tracker looks up the corresponding instance id for each sumbap.
 */
// TODO(schmluk): Clean submap creation away from id tracking.
class DetectronIDTracker : public IDTrackerBase {
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

    float vx = 320;
    float vy = 240;
    float fx = 320;
    float fy = 320;

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

 private:
  static config_utilities::Factory::RegistrationRos<
      IDTrackerBase, DetectronIDTracker, std::shared_ptr<LabelHandler>>
      registration_;
  const Config config_;
  std::unordered_map<int, int> unknown_ids;      // for error handling

  // Tracking information.
  bool is_initialized_ = false;
  Transformation T_M_C_prev_;
  cv::Mat id_image_prev_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_DETECTRON_ID_TRACKER_H_
