#ifndef PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_
#define PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_

#include <panoptic_mapping/preprocessing/detectron_id_tracker.h>
#include <panoptic_mapping_msgs/DetectronLabel.h>
#include <panoptic_mapping_msgs/DetectronLabels.h>

namespace panoptic_mapping {

DetectronIDTracker::DetectronLabel detectronLabelFromMsg(
    const panoptic_mapping_msgs::DetectronLabel& msg);
DetectronIDTracker::DetectronLabels detectronLabelsFromMsg(
    const panoptic_mapping_msgs::DetectronLabels& msg);

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_
