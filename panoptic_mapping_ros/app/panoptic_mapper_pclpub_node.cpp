#include <glog/logging.h>
#include <ros/ros.h>
#include <voxblox_ros/ptcloud_vis.h>
#include "panoptic_mapping_ros/panoptic_mapper.h"

/* This demo shows how to lookup data from the panotpic mapper which is running
 * in the background and e.g. publish the mesh of all submaps as pointcloud. */

void publishPointclouds(const panoptic_mapping::SubmapCollection& submaps,
                        const ros::Publisher& pub,
                        const voxblox::ExponentialOffsetIdColorMap& color_map) {
  // This function parses through the submap collection and creates a pointcloud
  // for each submap.
  // visualization_msgs::MarkerArray marker_array; 

  for (const panoptic_mapping::Submap& submap : submaps) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = submap.getFrameName();
    marker.header.stamp = ros::Time::now();
    marker.ns = submap.getName();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    // Get all verteces in the mesh.
    voxblox::BlockIndexList index_list;
    submap.getMeshLayer().getAllAllocatedMeshes(&index_list);
    // submap.getMeshLayer().getAllUpdatedMeshes(&index_list);
    for (const voxblox::BlockIndex& index : index_list) {
      
      auto mesh = submap.getMeshLayer().getMeshPtrByIndex(index);
      
      for (size_t i = 0u; i < mesh->vertices.size(); i++) {
        geometry_msgs::Point point_msg;
        tf::pointEigenToMsg(mesh->vertices[i].cast<double>(), point_msg);
        marker.points.push_back(point_msg);
        // // Set the submap color.
        // panoptic_mapping::Color color = color_map.colorLookup(submap.getID());
        // marker.color.r = color.r;
        // marker.color.g = color.g;
        // marker.color.b = color.b;
        // marker.color.a = 255;
        marker.colors.push_back(getVertexColor(mesh, voxblox::ColorMode::kColor, i));
      }
    }
    if (marker.points.empty()) {
      continue;
    }

    pub.publish(marker);
    // else {
    //  marker_array.markers.push_back(marker);
    // }
    // pub.publish(marker_array);
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
      nh.advertise<visualization_msgs::Marker>("/panoptic_mapper/submap_points", 100);
  voxblox::ExponentialOffsetIdColorMap color_map;
  int color_discretization;
  nh_private.getParam("visualization/submaps/submap_color_discretization",
                      color_discretization);
  color_map.setItemsPerRevolution(color_discretization);

  // Publish whenever there is an update.
  // double pcl_publish_time = 2.0;
  // double last_tick = ros::Time::now().toSec();
  while (ros::ok()) {
    //double now = ros::Time::now().toSec();
    if (map.wasUpdated()) {
      // last_tick = now;
      publishPointclouds(map.getSubmaps(), pcl_pub, color_map);
      
    }
    ros::spinOnce();
  }

  // ros::spin();


  return 0;
}
