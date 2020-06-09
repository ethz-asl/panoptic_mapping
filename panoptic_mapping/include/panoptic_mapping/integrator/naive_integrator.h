#ifndef PANOPTIC_MAPPING_INTEGRATOR_NAIVE_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATOR_NAIVE_INTEGRATOR_H_

#include <memory>

#include <voxblox/integrator/tsdf_integrator.h>

#include "panoptic_mapping/integrator/pointcloud_integrator_base.h"

namespace panoptic_mapping {

/**
 * Split the pointcloud by segmentation and use a voxblox tsdf_integrator to integrate each to its corresponding submap.
 */
class NaivePointcloudIntegrator : public PointcloudIntegratorBase {
 public:
  struct Config : PointcloudIntegratorBase::Config {
    // Does not need more than the voxblox cfg
    voxblox::TsdfIntegratorBase::Config voxblox_integrator_config;
  };

  NaivePointcloudIntegrator() = default;
  virtual ~NaivePointcloudIntegrator() = default;

  void setupFromConfig(PointcloudIntegratorBase::Config* config);

  // process a and integrate a pointcloud
  void processPointcloud(SubmapCollection *submaps,
                         const Transformation &T_M_C,
                         const Pointcloud &pointcloud,
                         const Colors &colors,
                         const std::vector<int> &ids) override;
 protected:
  Config config_;
  std::unique_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
};

}  // namespace panoptic_mapping

#endif // PANOPTIC_MAPPING_INTEGRATOR_NAIVE_INTEGRATOR_H_
