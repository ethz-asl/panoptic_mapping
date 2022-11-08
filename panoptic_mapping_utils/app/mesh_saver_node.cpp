#include <glog/logging.h>
#include <ros/ros.h>

#include "panoptic_mapping_utils/mesh_saver.h"

int main(int argc, char** argv) {
  // Start Ros.
  ros::init(argc, argv, "mesh_saver", ros::init_options::NoSigintHandler);


  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup node.
  ros::NodeHandle nh_private("~");
  panoptic_mapping::MeshSaver mesh_saver(nh_private);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
