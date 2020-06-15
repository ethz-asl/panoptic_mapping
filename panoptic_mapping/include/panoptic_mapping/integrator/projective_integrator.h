#ifndef PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_

#include <memory>

#include <voxblox/integrator/tsdf_integrator.h>

#include "panoptic_mapping/integrator/integrator_base.h"

namespace panoptic_mapping {

/**
 * Split the pointcloud by segmentation and use a voxblox tsdf_integrator to integrate each to its corresponding submap.
 */
class ProjectiveIntegrator : public IntegratorBase {
 public:
  struct Config : IntegratorBase::Config {
    // Does not need more than the voxblox cfg
    voxblox::TsdfIntegratorBase::Config voxblox_integrator_config;
    std::string voxblox_integrator_type = "fast"; // simple, merged, fast, projective
  };

  ProjectiveIntegrator() = default;
  virtual ~ProjectiveIntegrator() = default;

  void setupFromConfig(IntegratorBase::Config *config);

  // process a and integrate a pointcloud
  void processPointcloud(SubmapCollection *submaps,
                         const Transformation &T_M_C,
                         const Pointcloud &pointcloud,
                         const Colors &colors,
                         const std::vector<int> &ids) override;
 protected:
  Config config_;
};

}  // namespace panoptic_mapping

#endif // PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_
