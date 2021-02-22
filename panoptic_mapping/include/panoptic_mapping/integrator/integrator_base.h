#ifndef PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_BASE_H_
#define PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_BASE_H_

#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/input_data_user.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * Interface for TSDF integrators.
 */
class IntegratorBase : public InputDataUser {
 public:
  IntegratorBase() {
    // Per default require all three images.
    addRequiredInput(InputData::InputType::kDepthImage);
    addRequiredInput(InputData::InputType::kColorImage);
    addRequiredInput(InputData::InputType::kSegmentationImage);
  }
  ~IntegratorBase() override = default;

  virtual void processInput(SubmapCollection* submaps, InputData* input) = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATOR_INTEGRATOR_BASE_H_
