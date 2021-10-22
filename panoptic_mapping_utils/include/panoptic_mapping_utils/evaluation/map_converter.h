#ifndef PANOPTIC_MAPPING_UTILS_EVALUATION_MAP_CONVERTER_H_
#define PANOPTIC_MAPPING_UTILS_EVALUATION_MAP_CONVERTER_H_

#include <memory>
#include <string>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/3rd_party/nanoflann.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/tools/planning_interface.h>
#include <panoptic_mapping_ros/visualization/submap_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <voxblox_ros/tsdf_server.h>

namespace panoptic_mapping {

// Evaluation tools in a ROS node for visualization. Largely based on the
// voxblox_ros/voxblox_eval.cc code.
class MapConverter {
 public:
    MapConverter(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~MapConverter() = default;

 private:
  // ROS.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::shared_ptr<SubmapCollection> submaps_;
  };

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_UTILS_EVALUATION_MAP_CONVERTER_H_
