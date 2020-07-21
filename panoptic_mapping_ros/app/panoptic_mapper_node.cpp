#include "panoptic_mapping_ros/panoptic_mapper.h"
#include <ros/ros.h>
#include <glog/logging.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "panoptic_mapper", ros::init_options::NoSigintHandler);

  // Setup logging
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Run node
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  panoptic_mapping::PanopticMapper mapper(nh, nh_private);

  ros::spin();
  return 0;
}
