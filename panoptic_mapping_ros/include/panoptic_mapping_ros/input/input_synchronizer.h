#ifndef PANOPTIC_MAPPING_ROS_INPUT_INPUT_SYNCHRONIZER_H_
#define PANOPTIC_MAPPING_ROS_INPUT_INPUT_SYNCHRONIZER_H_

#include <atomic>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/input_data.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "panoptic_mapping_ros/input/input_subscriber.h"

namespace panoptic_mapping {

/**
 * @brief This class subscribes to all required input types via ROS and
 * synchronizes them into an input data package to be processed.
 */
class InputSynchronizer : public InputSynchronizerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 2;
    int max_input_queue_length = 10;  // Number of data points per type stored
    // before old data starts being discarded.
    std::string global_frame_name = "mission";
    std::string sensor_frame_name =
        "";  // Empty (default) take the frame of the depth message header.
    float transform_lookup_time =
        0.1f;  // s, Maximum time to wait for transforms.
    double max_delay = 0.0; // s, Maximum delay between Image messages that should be synced

    Config() { setConfigName("InputSynchronizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  InputSynchronizer(const Config& config, const ros::NodeHandle& nh);
  ~InputSynchronizer() override = default;

  // Access.
  /**
   * @brief Get the list of requested input types.
   *
   * @return const InputData::InputTypes&
   */
  const InputData::InputTypes& getRequestedInputs() const {
    return requested_inputs_;
  }

  /**
   * @brief Check whether there is any input data ready to be retrieved.
   */
  bool hasInputData() const { return data_is_ready_; }

  /**
   * @brief Extract the most recent input data from the queue. The data will be
   * deleted from the queue. This call is blocking.
   *
   * @return std::shared_ptr<InputData> The data. Nullptr if none is ready or
   * data lookup failed.
   */
  std::shared_ptr<InputData> getInputData();

  // Setup.
  /**
   * @brief Setup tool. Adds inputs types to the list of required inputs. First
   * add all inputs, then call 'advertiseInputTopics()'.
   *
   * @param types List of types to add to the required inputs.
   */
  void requestInputs(const InputData::InputTypes& types);

  /**
   * @brief Setup tool. Subscribes to all required inputs. Use 'requestInputs()'
   * first to add inputs, then advertise them.
   */
  void advertiseInputTopics();

 private:
  /**
   * @brief Utility function for more readable queue allocation.
   *
   * @tparam MsgT Message type of the topic to subscribe.
   * @param type Input type based on which the topic names are selected.
   * @param extraction_function Function that is called to extract the message
   * to input data.
   */
  template <typename MsgT>
  void addQueue(InputData::InputType type,
                std::function<void(const MsgT&, InputSynchronizerData*)>
                    extraction_function) {
    subscribers_.emplace_back(std::make_unique<InputSubscriber<MsgT>>(
        nh_, kDefaultTopicNames_.at(type), config_.max_input_queue_length,
        extraction_function, this));
  }

  /**
   * @brief Try to lookup the specified transform from tf.
   *
   * @param timestamp Timestamp to lookup.
   * @param base_frame Base frame to lookup.
   * @param child_frame Child frame to lookup.
   * @param transformation Output transform if it could be looked up.
   * @return True if the transform could be looked up.
   */
  bool lookupTransform(const ros::Time& timestamp,
                       const std::string& base_frame,
                       const std::string& child_frame,
                       Transformation* transformation) const;

  bool getDataInQueue(const ros::Time& timestamp,
                      InputSynchronizerData** data) override;

  bool allocateDataInQueue(const ros::Time& timestamp);

  void checkDataIsReady(InputSynchronizerData* data) override;

 private:
  template <typename T>
  friend class InputSubscriber;
  const Config config_;

  // ROS.
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;

  // Inputs.
  InputData::InputTypes requested_inputs_;
  InputData::InputTypes subscribed_inputs_;
  std::vector<std::unique_ptr<InputSubscriberBase>> subscribers_;

  // Data.
  std::vector<std::unique_ptr<InputSynchronizerData>> data_queue_;

  // Settings.
  static const std::unordered_map<InputData::InputType, std::string>
      kDefaultTopicNames_;

  // Variables.
  std::atomic<bool> data_is_ready_;
  ros::Time oldest_time_ = ros::Time(0);
  std::string used_sensor_frame_name_;
  std::mutex data_mutex_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_INPUT_INPUT_SYNCHRONIZER_H_
