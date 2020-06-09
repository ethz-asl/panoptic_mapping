#include "panoptic_mapping/integrator/pointcloud_integrator_factory.h"
#include "panoptic_mapping/integrator/naive_integrator.h"
//#include "panoptic_mapping/integrator/projective_multi_tsdf_integrator.h"


namespace panoptic_mapping {

std::unique_ptr<PointcloudIntegratorBase> PointcloudIntegratorFactory::create(const std::string &type) {
  if (type == "naive"){
    return std::unique_ptr<PointcloudIntegratorBase>(new NaivePointcloudIntegrator());
  } else if (type == "projective_multi_tsdf"){
    //return std::unique_ptr<PointcloudIntegratorBase>(new ProjectiveMutliTSDFIntegrator());
  } {
    LOG(WARNING) << "Unknown pointcloud integrator type '" << type << "' using 'naive' instead.";
    return std::unique_ptr<PointcloudIntegratorBase>(new NaivePointcloudIntegrator());
  }
}

}  // namespace panoptic_mapping