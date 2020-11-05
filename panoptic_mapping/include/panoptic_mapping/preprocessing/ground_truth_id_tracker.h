#ifndef PANOPTIC_MAPPING_PREPROCESSING_GROUND_TRUTH_ID_TRACKER_H_
#define PANOPTIC_MAPPING_PREPROCESSING_GROUND_TRUTH_ID_TRACKER_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "panoptic_mapping/preprocessing/id_tracker_base.h"
#include "panoptic_mapping/preprocessing/label_handler.h"

namespace panoptic_mapping {

/**
 * This id tracker looks up the corresponding instance id for each sumbap.
 */
class GroundTruthIDTracker : public IDTrackerBase {
 public:
  struct Config {
    double instance_voxel_size = 0.03;
    double background_voxel_size = 0.07;
    int voxels_per_side = 16;

    [[nodiscard]] Config isValid() const;
  };

  GroundTruthIDTracker(const Config& config,
                       std::shared_ptr<LabelHandler> label_handler);
  ~GroundTruthIDTracker() override = default;

  void processImages(SubmapCollection* submaps, const Transformation& T_M_C,
                     const cv::Mat& depth_image, const cv::Mat& color_image,
                     cv::Mat* id_image) override;

  void processPointcloud(SubmapCollection* submaps, const Transformation& T_M_C,
                         const Pointcloud& pointcloud, const Colors& colors,
                         std::vector<int>* ids) override;

 private:
  void allocateSubmap(int instance, SubmapCollection* submaps);
  void printAndResetWarnings();

 private:
  const Config config_;
  std::unordered_map<int, int> instance_to_id_;  // track active maps
  std::unordered_map<int, int> unknown_ids;      // for error handling
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_GROUND_TRUTH_ID_TRACKER_H_
