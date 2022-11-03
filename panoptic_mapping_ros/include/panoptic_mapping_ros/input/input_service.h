#ifndef PANOPTIC_MAPPING_ROS_INPUT_INPUT_SERVICE_H_
#define PANOPTIC_MAPPING_ROS_INPUT_INPUT_SERVICE_H_

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/input_data.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <panoptic_mapping_msgs/Input.h>
#include <tf/transform_datatypes.h>

#include "panoptic_mapping_ros/input/input_queue.h"

namespace panoptic_mapping {

/**
 * @brief This class is an alternantive, blockign input method based on service
 * calls.
 */
class InputService : public InputQueueBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 2;
    Config() { setConfigName("InputService"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  InputService(const Config& config, const ros::NodeHandle& nh);
  ~InputService() override = default;

  bool hasInputData();

  std::shared_ptr<InputData> getInputData();

  bool inputCallback(panoptic_mapping_msgs::Input::Request& request,
                     panoptic_mapping_msgs::Input::Response& response);
  void requestInputs(const InputData::InputTypes& types);
  /**
   * @brief Setup tool. Subscribes to all required inputs. Use 'requestInputs()'
   * first to add inputs, then advertise them.
   */
  void advertiseInputTopics();

 private:
  const Config config_;
  InputData::InputTypes requested_inputs_;
  bool has_data_ = false;
  std::shared_ptr<InputData> current_data_;
  ros::ServiceServer input_srv_;

  ros::NodeHandle nh_;

};
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_INPUT_INPUT_SERVICE_H_
