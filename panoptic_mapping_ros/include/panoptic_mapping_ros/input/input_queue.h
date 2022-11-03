#ifndef PANOPTIC_MAPPING_ROS_INPUT_INPUT_QUEUE_BASE_H_
#define PANOPTIC_MAPPING_ROS_INPUT_INPUT_QUEUE_BASE_H_
#include <panoptic_mapping/common/input_data.h>

namespace panoptic_mapping {

/**
 * @brief Define a base interface to input queues as required by
 * panoptic_mapper.h.
 */
class InputQueueBase {
 public:
  InputQueueBase() = default;
  virtual ~InputQueueBase() = default;
  virtual bool hasInputData() = 0;
  virtual std::shared_ptr<InputData> getInputData() = 0;
  virtual void requestInputs(const InputData::InputTypes& types) = 0;
  /**
   * @brief Setup tool. Subscribes to all required inputs. Use 'requestInputs()'
   * first to add inputs, then advertise them.
   */
  virtual void advertiseInputTopics() = 0;
};
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_INPUT_INPUT_QUEUE_BASE_H_
