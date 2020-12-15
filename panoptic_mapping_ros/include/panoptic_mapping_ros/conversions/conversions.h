#ifndef PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_
#define PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_

#include <string>
#include <unordered_map>

#include <panoptic_mapping_msgs/DetectronLabels.h>

namespace panoptic_mapping {

struct DetectronLabel {
  int id = 0;
  bool is_thing = true;
  int category_id = 0;
  int instance_id = 0;
  float score = 0.f;
};

typedef std::unordered_map<int, DetectronLabel> DetectronLabels;

DetectronLabel detectronLabelFromMsg(
    const panoptic_mapping_msgs::DetectronLabel& msg);
DetectronLabels detectronLabelsFromMsg(
    const panoptic_mapping_msgs::DetectronLabels& msg);

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_
