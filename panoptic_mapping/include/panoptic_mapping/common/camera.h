#ifndef PANOPTIC_MAPPING_COMMON_CAMERA_H_
#define PANOPTIC_MAPPING_COMMON_CAMERA_H_

#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * Utility class bundling camera related operations and data. Currently the
 * camera pose T_M_C is not stored with a camera but provided by the caller.
 */
class Camera {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 0;

    // Camera Intrinsics. [px]
    int width = 640;
    int height = 480;
    float vx = 320.f;  // Center point.
    float vy = 240.f;
    float fx = 320.f;  // Focal lengths.
    float fy = 320.f;

    float max_range = 5.f;   // m
    float min_range = 0.1f;  // m

    Config() { setConfigName("Camera"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit Camera(const Config& config);
  virtual ~Camera() = default;

  // Access.
  const Config& getConfig() const { return config_; }

  // Visibility checks.
  bool pointIsInViewFrustum(const Point& point_C,
                            float inflation_distance = 0.f) const;

  bool submapIsInViewFrustum(const Submap& submap,
                             const Transformation& T_M_C) const;

  bool blockIsInViewFrustum(const Submap& submap,
                            const voxblox::BlockIndex& block_index,
                            const Transformation& T_M_C) const;

  bool blockIsInViewFrustum(const Submap& submap,
                            const voxblox::BlockIndex& block_index,
                            const Transformation& T_C_S, float block_size,
                            float block_diag_half) const;

  // Visibility search.
  std::vector<int> findVisibleSubmapIDs(const SubmapCollection& submaps,
                                        const Transformation& T_M_C,
                                        bool only_active_submaps = true,
                                        bool include_freespace = false) const;

  voxblox::BlockIndexList findVisibleBlocks(const Submap& subamp,
                                            const Transformation& T_M_C) const;

  std::unordered_map<int, voxblox::BlockIndexList> findVisibleBlocks(
      const SubmapCollection& submaps, const Transformation& T_M_C,
      bool only_active_submaps = true) const;

  // Projection.
  bool projectPointToImagePlane(const Point& p_C, float* u, float* v) const;

  bool projectPointToImagePlane(const Point& p_C, int* u, int* v) const;

  cv::Mat computeVertexMap(const cv::Mat& depth_image) const;

  cv::Mat computeValidityImage(const cv::Mat& depth_image) const;

 private:
  const Config config_;

  // Pre-computed stored values.
  std::vector<Point> view_frustum_;  // Top, right, bottom, left plane normals.
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_CAMERA_H_
