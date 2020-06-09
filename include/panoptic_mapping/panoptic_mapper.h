#ifndef PANOPTIC_MAPPING_INCLUDE_PANOPTIC_MAPPING_PANOPTIC_MAPPER_H_
#define PANOPTIC_MAPPING_INCLUDE_PANOPTIC_MAPPING_PANOPTIC_MAPPER_H_

#include <string>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <voxblox_msgs/FilePath.h>
#include <voxgraph/frontend/map_tracker/transformers/tf_transformer.h>

#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/submap_collection.h"
#include "panoptic_mapping/preprocessing/label_handler.h"
#include "panoptic_mapping/integrator/pointcloud_integrator_base.h"
#include "panoptic_mapping/visualization/tsdf_visualizer.h"


namespace panoptic_mapping {

class PanopticMapper {
 public:
  PanopticMapper(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private);
  virtual ~PanopticMapper() = default;

  // ROS callbacks
  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);
  void publishMeshCallback(const ros::TimerEvent &);
  bool saveMapCallback(voxblox_msgs::FilePath::Request& request, voxblox_msgs::FilePath::Response& response);
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request, voxblox_msgs::FilePath::Response& response);
  bool setColoringModeCallback(voxblox_msgs::FilePath::Request& request, voxblox_msgs::FilePath::Response& response);

  // io
  bool saveMap(const std::string& file_path);
  bool loadMap(const std::string& file_path);

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers, Publishers, Services, Timers
  ros::Subscriber pointcloud_sub_;
  ros::Publisher mesh_pub_;
  ros::Publisher tsdf_blocks_pub_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer set_coloring_mode_srv_;
  ros::Timer mesh_timer_;

  // members
  SubmapCollection submaps_;
  LabelHandler label_handler_;
  std::unique_ptr<PointcloudIntegratorBase> pointcloud_integrator_;
  voxgraph::TfTransformer tf_transformer_;
  TsdfVisualizer tsdf_visualizer_;

  // params
  std::string coloring_mode_;   // 'color', 'normals', 'submaps', 'instances', set via setColoringMode()

  // methods
  // setup
  void setupROS();
  void setupMembers();

  // visualization
  void setColoringMode(const std::string& coloring_mode);
  void setSubmapColor(Submap* submap);
  void publishMeshes(bool force_update_all = false, bool force_mesh_all = false);
};

} // namespace panoptic_mapping

#endif //PANOPTIC_MAPPING_INCLUDE_PANOPTIC_MAPPING_PANOPTIC_MAPPER_H_
