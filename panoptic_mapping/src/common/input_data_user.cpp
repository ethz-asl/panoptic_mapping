#include "panoptic_mapping/common/input_data_user.h"

#include <string>
#include <unordered_set>

namespace panoptic_mapping {

bool InputDataUser::inputIsValid(const InputData& input_data,
                                 bool raise_warning) {
  // Check whether all inputs are set and otherwise raise a warning if
  // requested.
  std::string error_msg;
  for (const InputData::InputType& input : required_inputs_) {
    if (input_data.contained_inputs_.find(input) ==
        input_data.contained_inputs_.end()) {
      if (raise_warning) {
        if (error_msg.empty()) {
          error_msg = "The inputs '" + InputData::inputTypeToString(input);
        } else {
          error_msg += "', '" + InputData::inputTypeToString(input);
        }
      } else {
        return false;
      }
    }
  }
  if (raise_warning) {
    if (error_msg.empty()) {
      return true;
    }
    LOG(WARNING) << error_msg << "' are required and not provided.";
    return false;
  }
  return true;
}

void InputDataUser::addRequiredInput(InputData::InputType type) {
  required_inputs_.insert(type);
}

void InputDataUser::setRequiredInputs(
    const std::unordered_set<InputData::InputType>& types) {
  required_inputs_ = types;
}

}  // namespace panoptic_mapping
