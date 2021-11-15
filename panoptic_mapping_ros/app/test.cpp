#include <glog/logging.h>
#include <ros/ros.h>

#include "panoptic_mapping/map/classification/class_layer.h"

int main(int argc, char** argv) {
  // Start Ros.
  ros::init(argc, argv, "panoptic_mapper", ros::init_options::NoSigintHandler);

  return 0;
}
