#ifndef PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_
#define PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_

#include <memory>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/input_data_user.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/preprocessing/label_handler.h"

namespace panoptic_mapping {

/**
 * This class tracks and matches input to submap ids and allocates new submaps
 * where necessary.
 */
class IDTrackerBase : public InputDataUser {
 public:
  explicit IDTrackerBase(std::shared_ptr<LabelHandler> label_handler)
      : label_handler_(std::move(label_handler)) {
    // Per default require all three images.
    addRequiredInput(InputData::InputType::kDepthImage);
    addRequiredInput(InputData::InputType::kColorImage);
    addRequiredInput(InputData::InputType::kSegmentationImage);
  }
  ~IDTrackerBase() override = default;

  // Interface;
  virtual void processInput(SubmapCollection* submaps, InputData* input) = 0;

 protected:
  std::shared_ptr<LabelHandler> label_handler_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_ID_TRACKER_BASE_H_
