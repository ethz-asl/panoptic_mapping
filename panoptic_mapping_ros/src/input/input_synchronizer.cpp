#include "panoptic_mapping_ros/input/input_synchronizer.h"

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <cv_bridge/cv_bridge.h>
#include <minkindr_conversions/kindr_tf.h>
#include <panoptic_mapping_msgs/DetectronLabels.h>
#include <sensor_msgs/Image.h>

#include "panoptic_mapping_ros/conversions/conversions.h"
#include "panoptic_mapping_ros/input/input_queue.h"

namespace panoptic_mapping {

const std::unordered_map<InputData::InputType, std::string>
    InputSynchronizer::kDefaultTopicNames_ = {
        {InputData::InputType::kDepthImage, "depth_image_in"},
        {InputData::InputType::kColorImage, "color_image_in"},
        {InputData::InputType::kSegmentationImage, "segmentation_image_in"},
        {InputData::InputType::kDetectronLabels, "labels_in"}};

void InputSynchronizer::Config::checkParams() const {
  checkParamGT(max_input_queue_length, 0, "max_input_queue_length");
  checkParamCond(!global_frame_name.empty(),
                 "'global_frame_name' may not be empty.");
  checkParamCond(!sensor_frame_name.empty(),
                 "'sensor_frame_name' may not be empty.");
}

void InputSynchronizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("max_input_queue_length", &max_input_queue_length);
  setupParam("global_frame_name", &global_frame_name);
  setupParam("sensor_frame_name", &sensor_frame_name);
}

InputSynchronizer::InputSynchronizer(const Config& config,
                                     const ros::NodeHandle& nh)
    : config_(config.checkValid()), nh_(nh) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void InputSynchronizer::requestInputs(const InputData::InputTypes& types) {
  for (const auto& type : types) {
    requested_inputs_.insert(type);
  }
}

void InputSynchronizer::advertiseInputTopics() {
  // Parse all required inputs and allocate an input queue for each.
  // NOTE(schmluk): Image copies appear to be necessary since some of the data
  // is mutable and they get corrupted sometimes otherwise. Better be safe.
  for (const InputData::InputType type : requested_inputs_) {
    switch (type) {
      case InputData::InputType::kDepthImage: {
        using MsgT = sensor_msgs::ImageConstPtr;
        addQueue<MsgT>(type, [](const MsgT& msg, InputData* data) {
          cv_bridge::CvImageConstPtr depth = cv_bridge::toCvCopy(msg, "32FC1");
          data->setDepthImage(depth->image);
          data->setFrameName(msg->header.frame_id);
        });
        break;
      }
      case InputData::InputType::kColorImage: {
        using MsgT = sensor_msgs::ImageConstPtr;
        addQueue<MsgT>(type, [](const MsgT& msg, InputData* data) {
          cv_bridge::CvImageConstPtr color = cv_bridge::toCvCopy(msg, "bgr8");
          data->setColorImage(color->image);
        });
        break;
      }
      case InputData::InputType::kSegmentationImage: {
        using MsgT = sensor_msgs::ImageConstPtr;
        addQueue<MsgT>(type, [](const MsgT& msg, InputData* data) {
          cv_bridge::CvImageConstPtr seg = cv_bridge::toCvCopy(msg, "32SC1");
          data->setIdImage(seg->image);
        });
        break;
      }
      case InputData::InputType::kDetectronLabels: {
        using MsgT = panoptic_mapping_msgs::DetectronLabels;
        addQueue<MsgT>(type, [](const MsgT& msg, InputData* data) {
          data->setDetectronLabels(detectronLabelsFromMsg(msg));
        });
        break;
      }
    }
  }
}

void InputSynchronizer::checkForMatchingMessages(const ros::Time& timestamp) {
  // Check all data is available.
  for (const auto& queue : input_queues_) {
    if (!queue->hasTimeStamp(timestamp)) {
      return;
    }
  }

  // Collect the data from all queues.
  InputData data;
  for (const auto& queue : input_queues_) {
    queue->extractMsgToData(&data, timestamp);
  }

  // Get general data.
  tf::StampedTransform transform;
  const std::string sensor_frame_name = config_.sensor_frame_name.empty()
                                            ? data.sensorFrameName()
                                            : config_.sensor_frame_name;
  try {
    tf_listener_.lookupTransform(config_.global_frame_name, sensor_frame_name,
                                 timestamp, transform);
    Transformation T_M_C;
    tf::transformTFToKindr(transform, &T_M_C);
    data.setT_M_C(T_M_C);
  } catch (tf::TransformException& ex) {
    LOG_IF(WARNING, config_.verbosity > 0)
        << "Unable to lookup transform between '" << config_.sensor_frame_name
        << "' and '" << config_.global_frame_name << "' (" << ex.what()
        << "), skipping inputs.";
    return;
  }
  data.setTimeStamp(timestamp.toSec());

  // Send the input to its users.
  callback_(&data);
}

}  // namespace panoptic_mapping
