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
  checkParamGE(transform_lookup_time, 0.f, "transform_lookup_time");
}

void InputSynchronizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("max_input_queue_length", &max_input_queue_length);
  setupParam("global_frame_name", &global_frame_name);
  setupParam("sensor_frame_name", &sensor_frame_name);
  setupParam("use_transform_caching", &use_transform_caching);
  setupParam("transform_lookup_time", &transform_lookup_time);
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

  // Get the transform.
  if (!frames_setup_) {
    if (config_.sensor_frame_name.empty()) {
      used_sensor_frame_ = data.sensorFrameName();
    } else {
      used_sensor_frame_ = config_.sensor_frame_name;
    }
  }

  Transformation T_M_C;
  if (config_.use_transform_caching && frames_setup_) {
    const uint64_t time = timestamp.sec * 1000000000 + timestamp.nsec;
    auto it = transform_cache_.find(time);
    if (it == transform_cache_.end()) {
      // If the transform could not be cached a warning was already printed.
      return;
    } else {
      T_M_C = it->second;
      transform_cache_.erase(it);
    }
  } else {
    if (!lookupTransform(timestamp, config_.global_frame_name,
                         used_sensor_frame_, &T_M_C)) {
      return;
    }
  }
  data.setTimeStamp(timestamp.toSec());
  data.setT_M_C(T_M_C);
  frames_setup_ = true;

  // Send the input to its users.
  callback_(&data);
}

void InputSynchronizer::cacheTransform(const ros::Time& timestamp) {
  if (frames_setup_) {
    Transformation transform;
    if (lookupTransform(timestamp, config_.global_frame_name,
                        used_sensor_frame_, &transform)) {
      const uint64_t time = timestamp.sec * 1000000000 + timestamp.nsec;
      transform_cache_[time] = transform;
    }
  }
}

bool InputSynchronizer::lookupTransform(const ros::Time& timestamp,
                                        const std::string& base_frame,
                                        const std::string& child_frame,
                                        Transformation* transformation) const {
  // Try to lookup the transform for the maximum wait time.
  tf::StampedTransform transform;
  try {
    tf_listener_.waitForTransform(base_frame, child_frame, timestamp,
                                  ros::Duration(config_.transform_lookup_time));
    tf_listener_.lookupTransform(base_frame, child_frame, timestamp, transform);
  } catch (tf::TransformException& ex) {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Unable to lookup transform between '" << base_frame << "' and '"
        << child_frame << "' at time '" << timestamp << "' over '"
        << config_.transform_lookup_time << "s', skipping inputs. Exception: '"
        << ex.what() << "'.";
    return false;
  }
  CHECK_NOTNULL(transformation);
  Transformation T_M_C;
  tf::transformTFToKindr(transform, &T_M_C);
  *transformation = T_M_C;
  return true;
}

}  // namespace panoptic_mapping
