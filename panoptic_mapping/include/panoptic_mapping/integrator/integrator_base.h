#ifndef PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_BASE_H_
#define PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_BASE_H_

#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/core/submap_collection.h"
#include "panoptic_mapping/core/common.h"

namespace panoptic_mapping {

/**
 * Interface for pointcloud integrators.
 */
class IntegratorBase {
 public:
  // Each integrator is allowed to derive from Config and add params there
  struct Config {
    virtual ~Config() = default;
  };

  IntegratorBase() = default;
  virtual ~IntegratorBase() = default;

  // initialization function, these are allowed to pass PointcloudIntegratorBase::Config-ptrs to derived configs
  virtual void setupFromConfig(Config *config) = 0;

  // process a and integrate a pointcloud
  virtual void processPointcloud(SubmapCollection *submaps,
                                 const Transformation &T_M_C,
                                 const Pointcloud &pointcloud,
                                 const Colors &colors,
                                 const std::vector<int> &ids);

  // process a and integrate a pointcloud
  virtual void processImages(SubmapCollection *submaps,
                             const Transformation &T_M_C,
                             const cv::Mat &depth_image,
                             const cv::Mat &color_image,
                             const cv::Mat &id_image);
};

}  // namespace panoptic_mapping

#endif //PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_BASE_H_
