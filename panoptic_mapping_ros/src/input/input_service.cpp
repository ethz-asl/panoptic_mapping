#include "panoptic_mapping_ros/input/input_service.h"

#include <cv_bridge/cv_bridge.h>
#include <minkindr_conversions/kindr_tf.h>

#include "panoptic_mapping_ros/conversions/conversions.h"

namespace panoptic_mapping {
void InputService::Config::checkParams() const {}

void InputService::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
}

InputService::InputService(const Config& config, const ros::NodeHandle& nh)
    : config_(config.checkValid()), nh_(nh) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void InputService::requestInputs(const InputData::InputTypes& types) {
  for (const auto& type : types) {
    requested_inputs_.insert(type);
  }
}

void InputService::advertiseInputTopics() {
  input_srv_ =
      nh_.advertiseService("add_input", &InputService::inputCallback, this);
  LOG_IF(INFO, config_.verbosity >= 2) << "input service advertised";
}

bool InputService::hasInputData() {
  if (current_data_ && has_data_) {
    return true;
  }
  return false;
}

std::shared_ptr<InputData> InputService::getInputData() {
  std::shared_ptr<InputData> ret = nullptr;
  // return existing data, make space for new data
  ret.swap(current_data_);
  has_data_ = false;
  return ret;
}

bool InputService::inputCallback(
    panoptic_mapping_msgs::Input::Request& request,
    panoptic_mapping_msgs::Input::Response& response) {
  if (!request.blocking && hasInputData()) {
    // do not overwrite data, wait for it to be integrated into the map
    return false;
  }
  // create a new input collection
  current_data_ = std::make_shared<InputData>();

  for (const InputData::InputType type : requested_inputs_) {
    switch (type) {
      case InputData::InputType::kDepthImage: {
        const cv_bridge::CvImageConstPtr depth =
            cv_bridge::toCvCopy(request.depth_image, "32FC1");
        /*if (depth->empty()) {
          LOG_IF(INFO, config_.verbosity >= 1)
              << "Required depth image not found in service input.";
          return false;
        }*/
        current_data_->setDepthImage(depth->image);
        break;
      }
      case InputData::InputType::kColorImage: {
        const cv_bridge::CvImageConstPtr color =
            cv_bridge::toCvCopy(request.color_image, "bgr8");
        /*if (color->empty()) {
          LOG_IF(INFO, config_.verbosity >= 1)
              << "Required color image not found in service input.";
          return false;
        }*/
        current_data_->setColorImage(color->image);
        break;
      }
      case InputData::InputType::kSegmentationImage: {
        const cv_bridge::CvImageConstPtr seg =
            cv_bridge::toCvCopy(request.id_image, "32SC1");
        /*if (seg->empty()) {
          LOG_IF(INFO, config_.verbosity >= 1)
              << "Required id image not found in service input.";
          return false;
        }*/
        current_data_->setIdImage(seg->image);
        break;
      }
      case InputData::InputType::kUncertaintyImage: {
        const cv_bridge::CvImageConstPtr uncertainty =
            cv_bridge::toCvCopy(request.uncertainty_image, "32FC1");
        /*if (uncertainty->empty()) {
          LOG_IF(INFO, config_.verbosity >= 1)
              << "Required uncertainty image not found in service input.";
          return false;
        }*/
        current_data_->setUncertaintyImage(uncertainty->image);
        break;
      }
      default: {
        LOG_IF(INFO, config_.verbosity >= 1)
            << "Parser for required input type "
            << InputData::inputTypeToString(type) << " not implemented.";
        break;
      }
    }
  }
  // convert the pose
  tf::Transform tf_M_C;
  tf::transformMsgToTF(request.pose, tf_M_C);
  Transformation T_M_C;
  tf::transformTFToKindr(tf_M_C, &T_M_C);
  current_data_->setT_M_C(T_M_C);
  current_data_->setFrameName(request.sensor_frame_name);
  return true;
}

}  // namespace panoptic_mapping
