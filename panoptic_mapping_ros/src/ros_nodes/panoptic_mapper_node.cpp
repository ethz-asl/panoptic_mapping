#include "panoptic_mapping_ros/panoptic_mapper.h"
#include <ros/ros.h>
#include <glog/logging.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "panoptic_mapper", ros::init_options::NoSigintHandler);

  // Setup logging
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  //   VLOG(1) << "I'm printed when you run the program with --v=1 or higher";
  // Levels: 0 - None, 1 - Setup, 2 - Verbose, 3 - Detailed

  // Run node
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  panoptic_mapping::PanopticMapper mapper(nh, nh_private);

  ros::spin();
  return 0;
}