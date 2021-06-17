#include <glog/logging.h>
#include <ros/ros.h>

#include "panoptic_mapping_ros/panoptic_mapper.h"

/* This demo shows how to lookup data from the panotpic mapper which is running
 * in the background and e.g. publish the mesh of all submaps as pointcloud. */

void publishPointclouds(const panoptic_mapping::SubmapCollection& submaps,
                        const ros::Publisher& pub,
                        const voxblox::ExponentialOffsetIdColorMap& color_map) {
  // This function parses through the submap collection and creates a pointcloud
  // for each submap.
  for (const panoptic_mapping::Submap& submap : submaps) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = submap.getFrameName();
    marker.header.stamp = ros::Time::now();
    marker.ns = submap.getName();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    // Get all verteces in the mesh.
    voxblox::BlockIndexList index_list;
    submap.getMeshLayer().getAllAllocatedMeshes(&index_list);
    for (const voxblox::BlockIndex& index : index_list) {
      for (const panoptic_mapping::Point& vertex :
           submap.getMeshLayer().getMeshByIndex(index).vertices) {
        geometry_msgs::Point point;
        point.x = vertex.x();
        point.y = vertex.y();
        point.z = vertex.z();
        marker.points.push_back(point);
      }
    }
    if (marker.points.empty()) {
      return;
    }

    // Set the submap color.
    panoptic_mapping::Color color = color_map.colorLookup(submap.getID());
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = 255;

    // Publish the data.
    pub.publish(marker);
  }
}

int main(int argc, char** argv) {
  // Init ROS and arguments.
  ros::init(argc, argv, "panoptic_mapper", ros::init_options::NoSigintHandler);
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Run node.
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  panoptic_mapping::PanopticMapper mapper(nh, nh_private);

  // Get thread-safe read-only access to the map.
  const panoptic_mapping::ThreadSafeSubmapCollection& map =
      mapper.getThreadSafeSubmapCollection();

  // Advertise pointcloud topic and setup the color map.
  ros::Publisher pcl_pub =
      nh.advertise<visualization_msgs::Marker>("submap_points", 100);
  voxblox::ExponentialOffsetIdColorMap color_map;
  int color_discretization;
  nh_private.getParam("visualization/submaps/submap_color_discretization",
                      color_discretization);
  color_map.setItemsPerRevolution(color_discretization);

  // Publish whenever there is an update.
  while (ros::ok()) {
    if (map.wasUpdated()) {
      publishPointclouds(map.getSubmaps(), pcl_pub, color_map);
    }
    ros::spinOnce();
  }

  return 0;
}
