#ifndef PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_

#include <memory>
#include <string>

#include "panoptic_mapping/integrator/projection_interpolators.h"
#include "panoptic_mapping/integrator/integrator_base.h"
#include "panoptic_mapping/core/common.h"

namespace panoptic_mapping {

/**
 * Allocate blocks based on the image and project all visible blocks into the image for updates-
 */
class ProjectiveIntegrator : public IntegratorBase {
 public:
  struct Config : IntegratorBase::Config {
    // camera settings  [px]
    int width = 640;
    int height = 320;
    float vx = -1;  // defaults to the center
    float vy = -1;
    float focal_length = 320;

    // integration params
    float max_range = 5;   // m
    float min_range = 0.1;  // m
    bool use_weight_dropoff = true;
    bool use_constant_weight = false;
    bool foreign_rays_clear = true;   // observations of object B can clear spcae in object A
    float sparsity_compensation_factor = 1.0;
    float max_weight = 1e5;
    std::string interpolation_method;   // nearest, bilinear, adaptive, semantic

    // system params
    int integration_threads = 0;
  };

  ProjectiveIntegrator() = default;
  virtual ~ProjectiveIntegrator() = default;

  void setupFromConfig(IntegratorBase::Config *config) override;

  void processImages(SubmapCollection *submaps,
                     const Transformation &T_M_C,
                     const cv::Mat &depth_image,
                     const cv::Mat &color_image,
                     const cv::Mat &id_image) override;
 protected:
  // components
  Config config_;
  std::unique_ptr<InterpolatorBase> interpolator_;

  // variables
  Eigen::MatrixXf range_image_;
  float max_range_in_image_;

  // cached data
  std::vector<Point> view_frustum_;  // (top, right, bottom, left, plane normals)

  // methods
  void allocateNewBlocks(SubmapCollection *submaps,
                         const Transformation &T_M_C,
                         const cv::Mat &depth_image,
                         const cv::Mat &id_image);

  void findVisibleBlocks(const Submap &submap,
                         const Transformation &T_M_C,
                         voxblox::BlockIndexList *block_list);

  void updateTsdfBlock(const voxblox::BlockIndex &index,
                       Submap *submap,
                       const Transformation &T_M_C,
                       const cv::Mat &color_image,
                       const cv::Mat &id_image);

};

}  // namespace panoptic_mapping

#endif // PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_
