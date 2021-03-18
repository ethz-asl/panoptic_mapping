#ifndef PANOPTIC_MAPPING_ROS_INPUT_INPUT_QUEUE_H_
#define PANOPTIC_MAPPING_ROS_INPUT_INPUT_QUEUE_H_

#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <panoptic_mapping/common/input_data.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "panoptic_mapping_ros/input/input_synchronizer.h"

namespace panoptic_mapping {

/**
 * Tool to manage the subscription and and storage of each input topic.
 */

class InputQueueBase {
 public:
  InputQueueBase() = default;
  virtual ~InputQueueBase() = default;

  virtual bool hasTimeStamp(const ros::Time& stamp) = 0;
  virtual void extractMsgToData(InputData* data,
                                const ros::Time& timestamp) = 0;
};

// Some template specializations to lookup the timestamp of a message.
template <typename MsgT>
inline const ros::Time& getTimeStampFromMsg(const MsgT& msg) {
  return msg.header.stamp;
}

inline const ros::Time& getTimeStampFromMsg(
    const sensor_msgs::ImageConstPtr& msg) {
  return msg->header.stamp;
}

template <typename MsgT>
class InputQueue : public InputQueueBase {
 public:
  InputQueue(const ros::NodeHandle& nh, const std::string& topic_name,
             int queue_size,
             std::function<void(const MsgT&, InputData*)> extraction_function,
             std::function<void(const ros::Time&)> new_message_callback)
      : extraction_function_(std::move(extraction_function)),
        new_message_callback_(std::move(new_message_callback)),
        queue_size_(queue_size),
        nh_(nh) {
    // Subscribe to the topic.
    subscriber_ = nh_.subscribe(topic_name, queue_size_,
                                &InputQueue<MsgT>::msgCallback, this);
  }

  void msgCallback(const MsgT& msg) {
    // store the input message in the queue.
    msg_queue_.push_back(msg);
    if (msg_queue_.size() > queue_size_) {
      msg_queue_.pop_front();
    }
    new_message_callback_(getTimeStampFromMsg(msg));
  }

 private:
  bool hasTimeStamp(const ros::Time& timestamp) override {
    return std::find_if(msg_queue_.begin(), msg_queue_.end(),
                        [timestamp](const MsgT& msg) {
                          return getTimeStampFromMsg(msg) == timestamp;
                        }) != msg_queue_.end();
  }

  void extractMsgToData(InputData* data, const ros::Time& timestamp) override {
    auto it = std::find_if(msg_queue_.begin(), msg_queue_.end(),
                           [timestamp](const MsgT& msg) {
                             return getTimeStampFromMsg(msg) == timestamp;
                           });
    if (it == msg_queue_.end()) {
      LOG(WARNING) << "Tried to extract message for non-existing timestamp, no "
                      "data will be written.";
      return;
    }

    // Perform the extraction operation and pop the data from the queue.
    extraction_function_(*it, data);
    msg_queue_.erase(it);
  }

 private:
  // ROS Subscriber.
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  int queue_size_;

  // Message queue.
  std::deque<MsgT> msg_queue_;
  // Tells the queue how to store its data in the input data.
  std::function<void(const MsgT& msg, InputData* data)> extraction_function_;
  // Allow to call back to the parent to synchronize whenever new data arrives.
  std::function<void(const ros::Time& timestamp)> new_message_callback_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_INPUT_INPUT_QUEUE_H_
