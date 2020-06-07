#include "panoptic_mapping/input/pointcloud_integrator_factory.h"
#include "panoptic_mapping/input/naive_integrator.h"

namespace panoptic_mapping {

std::unique_ptr<PointcloudIntegratorBase> PointcloudIntegratorFactory::create(const std::string &type) {
  if (type == "naive"){
    return std::unique_ptr<PointcloudIntegratorBase>(new NaivePointcloudIntegrator());
  } else {
    LOG(WARNING) << "Unknown pointcloud integrator type '" << type << "' using 'naive' instead.";
    return std::unique_ptr<PointcloudIntegratorBase>(new NaivePointcloudIntegrator());
  }
}

}  // namespace panoptic_mapping