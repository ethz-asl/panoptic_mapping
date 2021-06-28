#ifndef PANOPTIC_MAPPING_ROS_INPUT_INPUT_SYNCHRONIZER_H_
#define PANOPTIC_MAPPING_ROS_INPUT_INPUT_SYNCHRONIZER_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/input_data.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "panoptic_mapping_ros/input/input_queue.h"

namespace panoptic_mapping {

/**
 * This class subscribes to all required input types via ROS and synchronizes
 * them into an input data package to be processed.
 */

class InputSynchronizer {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 2;
    int max_input_queue_length = 10;  // Number of data points per type stored
    // before old data starts being discarded.
    std::string global_frame_name = "mission";
    std::string sensor_frame_name = "";  // If no frame name is specified the
                                         // header of the depth image is taken.

    Config() { setConfigName("InputSynchronizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  InputSynchronizer(const Config& config, const ros::NodeHandle& nh);
  virtual ~InputSynchronizer() = default;

  // Access.
  const InputData::InputTypes& getRequestedInputs() const {
    return requested_inputs_;
  }

  // Setup tools. Add all inputs and set the callback, then advertise to setup.
  void requestInputs(const InputData::InputTypes& types);
  void setInputCallback(std::function<void(InputData*)> callback) {
    callback_ = std::move(callback);
  }
  void advertiseInputTopics();

 private:
  // Check for matching data to trigger the callback.
  void checkForMatchingMessages(const ros::Time& timestamp);

  // Utility function for more readable queue allocation.
  template <typename MsgT>
  void addQueue(
      InputData::InputType type,
      std::function<void(const MsgT&, InputData*)> extraction_function) {
    input_queues_.emplace_back(std::make_unique<InputQueue<MsgT>>(
        nh_, kDefaultTopicNames_.at(type), config_.max_input_queue_length,
        extraction_function, [this](const ros::Time& timestamp) {
          this->checkForMatchingMessages(timestamp);
        }));
  }

 private:
  const Config config_;

  // ROS.
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;

  // Inputs.
  InputData::InputTypes requested_inputs_;
  std::function<void(InputData*)> callback_;
  std::vector<std::unique_ptr<InputQueueBase>> input_queues_;

  // Settings.
  static const std::unordered_map<InputData::InputType, std::string>
      kDefaultTopicNames_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_INPUT_INPUT_SYNCHRONIZER_H_
