#include "panoptic_mapping/registration/tsdf_registrator.h"

namespace panoptic_mapping {

TsdfRegistrator::Config TsdfRegistrator::Config::isValid() const {
  return Config(*this);
}

TsdfRegistrator::TsdfRegistrator(const Config& config)
    : config_(config.isValid()) {}

bool TsdfRegistrator::checkSubmapAlignment(Submap* reference, Submap* other) {}

}  // namespace panoptic_mapping
