#include "panoptic_mapping/integrator/pointcloud_integrator_factory.h"

#include "panoptic_mapping/integrator/naive_integrator.h"
#include "panoptic_mapping/integrator/projective_multi_tsdf_integrator.h"

namespace panoptic_mapping {

std::unique_ptr<PointcloudIntegratorBase> PointcloudIntegratorFactory::create(std::string *type) {
  if (*type == "projective_multi_tsdf") {
    return std::unique_ptr<PointcloudIntegratorBase>(new ProjectiveMutliTSDFIntegrator());
  } else {
    // default to naive, but send a warning
    if (*type != "naive") {
      LOG(WARNING) << "Unknown pointcloud integrator type '" << *type << "' using 'naive' instead.";
      *type = "naive";
    }
    return std::unique_ptr<PointcloudIntegratorBase>(new NaivePointcloudIntegrator());
  }
}

}  // namespace panoptic_mapping