#include "panoptic_mapping/integrator/integrator_factory.h"

#include "panoptic_mapping/integrator/naive_integrator.h"
#include "panoptic_mapping/integrator/projective_integrator.h"

namespace panoptic_mapping {

std::unique_ptr<IntegratorBase> IntegratorFactory::create(std::string *type) {
  if (*type == "projective") {
    return std::unique_ptr<IntegratorBase>(new ProjectiveIntegrator());
  }  else {
    // default to naive, but send a warning
    if (*type != "naive") {
      LOG(WARNING) << "Unknown pointcloud integrator type '" << *type << "' using 'naive' instead.";
      *type = "naive";
    }
    return std::unique_ptr<IntegratorBase>(new NaiveIntegrator());
  }
}

}  // namespace panoptic_mapping