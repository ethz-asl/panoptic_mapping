#ifndef PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_FACTORY_H_
#define PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_FACTORY_H_

#include <memory>

#include "panoptic_mapping/integrator/integrator_base.h"

namespace panoptic_mapping {

/**
 * Class to produce tsdf integrators
 */
class IntegratorFactory {
 public:
  static std::unique_ptr<IntegratorBase> create(std::string *type);

 private:
  IntegratorFactory() = default;
  ~IntegratorFactory() = default;
};

}  // namespace panoptic_mapping

#endif //PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_FACTORY_H_
