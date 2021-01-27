#ifndef PANOPTIC_MAPPING_COMMON_INPUT_DATA_USER_H_
#define PANOPTIC_MAPPING_COMMON_INPUT_DATA_USER_H_

#include <unordered_set>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/input_data.h"

namespace panoptic_mapping {

/**
 * Utility base class that manages interactions with input data generation and
 * checking for modules using the InputData.
 */
class InputDataUser {
 public:
  InputDataUser() = default;
  virtual ~InputDataUser() = default;

  const std::unordered_set<InputData::InputType>& getRequiredInputs() const {
    return required_inputs_;
  }
  bool inputIsValid(const InputData& input_data, bool raise_warning = true);

 protected:
  void addRequiredInput(InputData::InputType type);
  void setRequiredInputs(const std::unordered_set<InputData::InputType>& types);

 private:
  std::unordered_set<InputData::InputType> required_inputs_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_INPUT_DATA_USER_H_
