#include "panoptic_mapping_ros/input/input_synchronizer.h"

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <cv_bridge/cv_bridge.h>
#include <minkindr_conversions/kindr_tf.h>
#include <panoptic_mapping_msgs/DetectronLabels.h>
#include <sensor_msgs/Image.h>

#include "panoptic_mapping_ros/conversions/conversions.h"

namespace panoptic_mapping {

const std::unordered_map<InputData::InputType, std::string>
    InputSynchronizer::kDefaultTopicNames_ = {
        {InputData::InputType::kDepthImage, "depth_image_in"},
        {InputData::InputType::kColorImage, "color_image_in"},
        {InputData::InputType::kSegmentationImage, "segmentation_image_in"},
        {InputData::InputType::kDetectronLabels, "labels_in"},
        {InputData::InputType::kUncertaintyImage, "uncertainty_image_in"}};

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
  setupParam("transform_lookup_time", &transform_lookup_time);
  setupParam("max_delay", &max_delay);
}

InputSynchronizer::InputSynchronizer(const Config& config,
                                     const ros::NodeHandle& nh)
    : config_(config.checkValid()), nh_(nh), data_is_ready_(false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  if (!config_.sensor_frame_name.empty()) {
    used_sensor_frame_name_ = config_.sensor_frame_name;
  }
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
  // NOTE(schmluk): each input writes to a different image so we can do this
  // concurrently, only lock the mutex when writing to the contained inputs.
  subscribers_.clear();
  subscribed_inputs_.clear();
  for (const InputData::InputType type : requested_inputs_) {
    switch (type) {
      case InputData::InputType::kDepthImage: {
        using MsgT = sensor_msgs::ImageConstPtr;
        addQueue<MsgT>(
            type, [this](const MsgT& msg, InputSynchronizerData* data) {
              const cv_bridge::CvImageConstPtr depth =
                  cv_bridge::toCvCopy(msg, "32FC1");
              data->data->depth_image_ = depth->image;

              // NOTE(schmluk): If the sensor frame name is not set
              // recover it from the depth image.
              if (this->used_sensor_frame_name_.empty()) {
                this->used_sensor_frame_name_ = msg->header.frame_id;
              }

              const std::lock_guard<std::mutex> lock(data->write_mutex_);
              data->data->contained_inputs_.insert(
                  InputData::InputType::kDepthImage);
            });
        subscribed_inputs_.insert(InputData::InputType::kDepthImage);
        break;
      }
      case InputData::InputType::kColorImage: {
        using MsgT = sensor_msgs::ImageConstPtr;
        addQueue<MsgT>(type, [](const MsgT& msg, InputSynchronizerData* data) {
          const cv_bridge::CvImageConstPtr color =
              cv_bridge::toCvCopy(msg, "bgr8");
          data->data->color_image_ = color->image;
          const std::lock_guard<std::mutex> lock(data->write_mutex_);
          data->data->contained_inputs_.insert(
              InputData::InputType::kColorImage);
        });
        subscribed_inputs_.insert(InputData::InputType::kColorImage);
        break;
      }
      case InputData::InputType::kSegmentationImage: {
        using MsgT = sensor_msgs::ImageConstPtr;
        addQueue<MsgT>(type, [](const MsgT& msg, InputSynchronizerData* data) {
          const cv_bridge::CvImageConstPtr seg =
              cv_bridge::toCvCopy(msg, "32SC1");
          data->data->id_image_ = seg->image;
          const std::lock_guard<std::mutex> lock(data->write_mutex_);
          data->data->contained_inputs_.insert(
              InputData::InputType::kSegmentationImage);
        });
        subscribed_inputs_.insert(InputData::InputType::kSegmentationImage);
        break;
      }
      case InputData::InputType::kDetectronLabels: {
        using MsgT = panoptic_mapping_msgs::DetectronLabels;
        addQueue<MsgT>(type, [](const MsgT& msg, InputSynchronizerData* data) {
          data->data->detectron_labels_ = detectronLabelsFromMsg(msg);
          const std::lock_guard<std::mutex> lock(data->write_mutex_);
          data->data->contained_inputs_.insert(
              InputData::InputType::kDetectronLabels);
        });
        subscribed_inputs_.insert(InputData::InputType::kDetectronLabels);
        break;
      }
      case InputData::InputType::kUncertaintyImage: {
            using MsgT = sensor_msgs::ImageConstPtr;
            addQueue<MsgT>(
                    type, [this](const MsgT& msg, InputSynchronizerData* data) {
                        const cv_bridge::CvImageConstPtr uncertainty =
                                cv_bridge::toCvCopy(msg, "32FC1");
                        data->data->uncertainty_image_ = uncertainty->image;
                        const std::lock_guard<std::mutex> lock(data->write_mutex_);
                        data->data->contained_inputs_.insert(
                                InputData::InputType::kUncertaintyImage);
                    });
            subscribed_inputs_.insert(InputData::InputType::kUncertaintyImage);
            break;
        }
    }
  }
}

bool InputSynchronizer::getDataInQueue(const ros::Time& timestamp,
                                       InputSynchronizerData** data) {
  // These are common operations for all subscribers so mutex them to avoid race
  // conditions.
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Check the data is still relevant to the queue.
  if (timestamp < oldest_time_) {
    return false;
  }
  double max_delay = config_.max_delay;
  auto it = find_if(
      data_queue_.begin(), data_queue_.end(),
      [&timestamp, &max_delay](const auto& arg) { return abs(arg->timestamp.toSec() - timestamp.toSec()) <= max_delay; });
  if (it != data_queue_.end()) {
    // There already exists a data point.
    if (!it->get()->valid) {
      return false;
    }
    *data = it->get();
    return true;
  }

  // Create a new data point.
  if (allocateDataInQueue(timestamp)) {
    *data = data_queue_.back().get();
    return true;
  }
  return false;
}

bool InputSynchronizer::allocateDataInQueue(const ros::Time& timestamp) {
  // NOTE(schmluk): This obviously modifies the queue but the data_mutex_ should
  // already be locked from the calling getDataInQueue().
  // Check max queue size.
  if (data_queue_.size() > config_.max_input_queue_length) {
    std::sort(data_queue_.begin(), data_queue_.end(),
              [](const auto& lhs, const auto& rhs) -> bool {
                return lhs->timestamp < rhs->timestamp;
              });
    data_queue_.erase(data_queue_.begin());
    data_is_ready_ = false;
    for (size_t i = 0; i < data_queue_.size(); ++i) {
      if (data_queue_[i]->ready) {
        data_is_ready_ = true;
        break;
      }
    }
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Input queue is getting too long, dropping oldest data.";
    oldest_time_ = data_queue_.front()->timestamp;
  }

  data_queue_.emplace_back(new InputSynchronizerData());
  InputSynchronizerData& data = *data_queue_.back();

  // Check transform.
  Transformation T_M_C;
  if (!used_sensor_frame_name_.empty()) {
    if (!lookupTransform(timestamp, config_.global_frame_name,
                         used_sensor_frame_name_, &T_M_C)) {
      data.valid = false;
      return false;
    }
  }

  // Allocate new data.
  data.data = std::make_shared<InputData>();
  data.data->setTimeStamp(timestamp.toSec());
  data.data->setT_M_C(T_M_C);
  data.data->setFrameName(used_sensor_frame_name_);
  data.timestamp = timestamp;
  return true;
}

void InputSynchronizer::checkDataIsReady(InputSynchronizerData* data) {
  const std::lock_guard<std::mutex> lock(data->write_mutex_);
  for (const InputData::InputType input : subscribed_inputs_) {
    if (!data->data->has(input)) {
      return;
    }
  }
  // Has all required inputs.
  data->ready = true;
  data_is_ready_ = true;
}

std::shared_ptr<InputData> InputSynchronizer::getInputData() {
  std::shared_ptr<InputData> result = nullptr;
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Get the first datum that is ready.
  std::sort(data_queue_.begin(), data_queue_.end(),
            [](const auto& lhs, const auto& rhs) -> bool {
              return lhs->timestamp < rhs->timestamp;
            });
  for (size_t i = 0; i < data_queue_.size(); ++i) {
    if (data_queue_[i]->ready) {
      // In case the sensor frame name is taken from the depth message check it
      // was written. This only happens for the first message.
      if (data_queue_[i]->data->sensorFrameName().empty()) {
        Transformation T_M_C;
        if (!lookupTransform(data_queue_[i]->timestamp,
                             config_.global_frame_name, used_sensor_frame_name_,
                             &T_M_C)) {
          return result;
        }
        data_queue_[i]->data->setT_M_C(T_M_C);
        data_queue_[i]->data->setFrameName(used_sensor_frame_name_);
      }

      // Get the result and erase from the queue.
      result = data_queue_[i]->data;
      data_queue_.erase(data_queue_.begin() + i);
      oldest_time_ = data_queue_.front()->timestamp;
      break;
    }
  }

  // Check whether there are other ready data points.
  for (size_t i = 0; i < data_queue_.size(); ++i) {
    if (data_queue_[i]->ready) {
      return result;
    }
  }
  data_is_ready_ = false;
  return result;
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
