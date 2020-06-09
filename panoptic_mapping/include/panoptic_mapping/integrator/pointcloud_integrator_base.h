#ifndef PANOPTIC_MAPPING_INTEGRATOR_POINTCLOUD_INTEGRATOR_BASE_H_
#define PANOPTIC_MAPPING_INTEGRATOR_POINTCLOUD_INTEGRATOR_BASE_H_

#include <vector>

#include <ros/node_handle.h>

#include "panoptic_mapping/core/submap_collection.h"
#include "panoptic_mapping/core/common.h"

namespace panoptic_mapping {

/**
 * Interface for pointcloud integrators.
 */
class PointcloudIntegratorBase {
 public:
  // Each integrator is allowed to derive from Config and add params there
  struct Config{
    virtual ~Config() = default;
  };

  PointcloudIntegratorBase() = default;
  virtual ~PointcloudIntegratorBase() = default;

  // initialization function, these are allowed to pass PointcloudIntegratorBase::Config-ptrs to derived configs
  virtual void setupFromConfig(Config* config) = 0;

  // process a and integrate a pointcloud
  virtual void processPointcloud(SubmapCollection *submaps,
                                 const Transformation &T_M_C,
                                 const Pointcloud &pointcloud,
                                 const Colors &colors,
                                 const std::vector<int> &ids) = 0;
};

}  // namespace panoptic_mapping

#endif //PANOPTIC_MAPPING_INTEGRATOR_POINTCLOUD_INTEGRATOR_BASE_H_
