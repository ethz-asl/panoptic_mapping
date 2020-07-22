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

#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/submap_collection.h"
#include "panoptic_mapping/preprocessing/label_handler.h"
#include "panoptic_mapping/integrator/integrator_base.h"

#include "panoptic_mapping_ros/visualization/tsdf_visualizer.h"

namespace panoptic_mapping {

class PanopticMapper {
 public:
  struct Config {
    int max_image_queue_length = 10;  // after this many images are queued for
    // integration start discarding old ones.

    // visualization
    std::string coloring_mode =
        "color";  // 'color', 'normals', 'submaps', 'instances', set via
    // setColoringMode()
    bool visualize_mesh = true;
    bool visualize_tsdf_blocks = false;
  };

  PanopticMapper(const ::ros::NodeHandle& nh,
                 const ::ros::NodeHandle& nh_private);
  virtual ~PanopticMapper() = default;

  // ROS callbacks
  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);
  void depthImageCallback(const sensor_msgs::ImagePtr& msg);
  void colorImageCallback(const sensor_msgs::ImagePtr& msg);
  void segmentationImageCallback(const sensor_msgs::ImagePtr& msg);
  void publishMeshCallback(const ros::TimerEvent&);
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
  ros::Timer mesh_timer_;

  // members
  Config config_;
  SubmapCollection submaps_;
  LabelHandler label_handler_;
  std::unique_ptr<IntegratorBase> tsdf_integrator_;
  voxgraph::TfTransformer tf_transformer_;
  TsdfVisualizer tsdf_visualizer_;

  // input processing
  std::deque<sensor_msgs::ImagePtr> depth_queue_;
  std::deque<sensor_msgs::ImagePtr> color_queue_;
  std::deque<sensor_msgs::ImagePtr> segmentation_queue_;

  // methods
  // setup
  void setupConfigFromRos();
  void setupRos();
  void setupMembers();

  // process input images and pointcloud
  void findMatchingMessagesToPublish(
      const sensor_msgs::ImagePtr& reference_msg);

  void processImages(const sensor_msgs::ImagePtr& depth_img,
                     const sensor_msgs::ImagePtr& color_img,
                     const sensor_msgs::ImagePtr& segmentation_img);

  void allocateSubmaps(const std::vector<int>& ids);

  // visualization
  void setColoringMode(const std::string& coloring_mode);

  void setSubmapColor(Submap* submap);

  void publishMeshes(bool force_update_all = false,
                     bool force_mesh_all = false);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
