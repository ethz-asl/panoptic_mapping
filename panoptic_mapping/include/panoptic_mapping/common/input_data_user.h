#ifndef PANOPTIC_MAPPING_COMMON_INPUT_DATA_USER_H_
#define PANOPTIC_MAPPING_COMMON_INPUT_DATA_USER_H_

#include <unordered_set>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/input_data.h"

namespace panoptic_mapping {

/**
 * @brief Utility base class that manages interactions with input data
 * generation and checking, for modules using the InputData.
 */
class InputDataUser {
 public:
  InputDataUser() = default;
  virtual ~InputDataUser() = default;

  /**
   * @brief Get the current set of required inputs.
   */
  const InputData::InputTypes& getRequiredInputs() const {
    return required_inputs_;
  }

  /**
   * @brief Check whether the provided set of inputs matches all requirements of
   * this module.
   *
   * @param input_data Input data to check.
   * @param raise_warning Whether to log which data are missing if any.
   * @return Whether all required inputs are contained in the data.
   */
  bool inputIsValid(const InputData& input_data, bool raise_warning = true);

 protected:
  /**
   * @brief Add an input type to the set of required inputs. Duplicates are
   * automatically managed.
   *
   * @param type Input to add.
   */
  void addRequiredInput(InputData::InputType type);

  /**
   * @brief Add mulitple input types to the set of required inputs. Duplicates
   * are automatically managed.
   *
   * @param types Inputs to add.
   */
  void addRequiredInputs(const InputData::InputTypes& types);

  /**
   * @brief Sets the required inputs, erasing all previously stored input types.
   *
   * @param types Inputs to add.
   */
  void setRequiredInputs(const InputData::InputTypes& types);

 private:
  InputData::InputTypes required_inputs_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_INPUT_DATA_USER_H_
