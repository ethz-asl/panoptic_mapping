#ifndef PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
#define PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <voxblox_msgs/FilePath.h>
#include <voxgraph/frontend/map_tracker/transformers/tf_transformer.h>

#include <panoptic_mapping/core/submap.h>
#include <panoptic_mapping/core/submap_collection.h>
#include <panoptic_mapping/integrator/integrator_base.h>
#include <panoptic_mapping/preprocessing/id_tracker_base.h>
#include <panoptic_mapping/preprocessing/label_handler.h>
#include <panoptic_mapping/registration/tsdf_registrator.h>

#include "panoptic_mapping_ros/visualization/submap_visualizer.h"

namespace panoptic_mapping {

class PanopticMapper {
 public:
  struct Config {
    int verbosity = 2;
    int max_image_queue_length = 10;  // after this many images are queued for
    // integration start discarding old ones.
    std::string global_frame_name = "mission";
    double visualization_interval = 1.0;  // s
  };

  PanopticMapper(const ::ros::NodeHandle& nh,
                 const ::ros::NodeHandle& nh_private);
  virtual ~PanopticMapper() = default;

  // ROS callbacks
  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);
  void depthImageCallback(const sensor_msgs::ImagePtr& msg);
  void colorImageCallback(const sensor_msgs::ImagePtr& msg);
  void segmentationImageCallback(const sensor_msgs::ImagePtr& msg);
  void publishVisualizationCallback(const ros::TimerEvent&);
  bool saveMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool setColoringModeCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT

  // io
  bool saveMap(const std::string& file_path);
  bool loadMap(const std::string& file_path);

  // visualization
  void publishMeshes();
  void publishTsdfBlocks();

 private:
  // setup
  void setupConfigFromRos();
  void setupRos();
  void setupMembers();

  // input processing
  void findMatchingMessagesToPublish(
      const sensor_msgs::ImagePtr& reference_msg);
  void processImages(const sensor_msgs::ImagePtr& depth_img,
                     const sensor_msgs::ImagePtr& color_img,
                     const sensor_msgs::ImagePtr& segmentation_img);

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers, Publishers, Services, Timers
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber depth_image_sub_;
  ros::Subscriber color_image_sub_;
  ros::Subscriber segmentation_image_sub_;
  ros::Publisher mesh_pub_;
  ros::Publisher tsdf_blocks_pub_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer set_coloring_mode_srv_;
  ros::Timer visualization_timer_;

  // members
  Config config_;
  SubmapCollection submaps_;
  voxgraph::TfTransformer tf_transformer_;
  std::shared_ptr<LabelHandler> label_handler_;
  std::unique_ptr<IntegratorBase> tsdf_integrator_;
  std::unique_ptr<IDTrackerBase> id_tracker_;
  std::unique_ptr<SubmapVisualizer> submap_visualizer_;
  std::unique_ptr<TsdfRegistrator> tsdf_registrator_;

  // input processing
  std::deque<sensor_msgs::ImagePtr> depth_queue_;
  std::deque<sensor_msgs::ImagePtr> color_queue_;
  std::deque<sensor_msgs::ImagePtr> segmentation_queue_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
