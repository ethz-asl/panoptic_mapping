#ifndef PANOPTIC_MAPPING_INTEGRATOR_NAIVE_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATOR_NAIVE_INTEGRATOR_H_

#include <memory>
#include <string>
#include <vector>

#include <voxblox/integrator/tsdf_integrator.h>

#include "panoptic_mapping/integrator/integrator_base.h"

namespace panoptic_mapping {

/**
 * Split the pointcloud by segmentation and use a voxblox tsdf_integrator to
 * integrate each to its corresponding submap.
 */
class NaiveIntegrator : public IntegratorBase {
 public:
  struct Config {
    // Does not need more than the voxblox cfg
    voxblox::TsdfIntegratorBase::Config voxblox_integrator_config;

    // integrator types: simple, merged, fast, projective
    std::string voxblox_integrator_type = "fast";

    Config isValid() const;
  };

  explicit NaiveIntegrator(const Config& config);
  ~NaiveIntegrator() override = default;

  // process a and integrate a pointcloud
  void processPointcloud(SubmapCollection* submaps, const Transformation& T_M_C,
                         const Pointcloud& pointcloud, const Colors& colors,
                         const std::vector<int>& ids) override;

 protected:
  const Config config_;
  std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATOR_NAIVE_INTEGRATOR_H_
