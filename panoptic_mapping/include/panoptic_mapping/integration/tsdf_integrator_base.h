#ifndef PANOPTIC_MAPPING_INTEGRATION_TSDF_INTEGRATOR_BASE_H_
#define PANOPTIC_MAPPING_INTEGRATION_TSDF_INTEGRATOR_BASE_H_

#include <memory>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/globals.h"
#include "panoptic_mapping/common/input_data_user.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * Interface for TSDF integrators.
 */
class TsdfIntegratorBase : public InputDataUser {
 public:
  explicit TsdfIntegratorBase(std::shared_ptr<Globals> globals)
      : globals_(std::move(globals)) {
    // Per default require all three images.
    addRequiredInput(InputData::InputType::kDepthImage);
    addRequiredInput(InputData::InputType::kColorImage);
    addRequiredInput(InputData::InputType::kSegmentationImage);
  }
  ~TsdfIntegratorBase() override = default;

  virtual void processInput(SubmapCollection* submaps, InputData* input) = 0;

 protected:
  std::shared_ptr<Globals> globals_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATION_TSDF_INTEGRATOR_BASE_H_
