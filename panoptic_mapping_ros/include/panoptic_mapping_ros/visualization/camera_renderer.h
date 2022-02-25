#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_CAMERA_RENDERER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_CAMERA_RENDERER_H_

#include <memory>
#include <string>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/common/globals.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/tools/planning_interface.h>
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "panoptic_mapping_msgs/RenderCameraImage.h"

namespace panoptic_mapping {

class CameraRenderer {
 public:
  // config
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    std::string ros_namespace;

    Config() { setConfigName("CameraRenderer"); }

   protected:
    void setupParamsAndPrinting() override;
    void fromRosParam() override;
    void checkParams() const override;
  };

  // Constructors.
  explicit CameraRenderer(const Config& config,
                          std::shared_ptr<Globals> globals,
                          std::shared_ptr<Camera> camera, bool print_config,
                          ros::NodeHandle nh);
  virtual ~CameraRenderer() = default;

  // Interaction.
  void setGlobalFrameName(const std::string& frame_name) {
    global_frame_name_ = frame_name;
  }
  void renderCameraView(SubmapCollection* submaps,
                        const tf::StampedTransform transform,
                        cv::Mat* rendered_image);

 private:
  const Config config_;

  // Members.
  std::shared_ptr<Globals> globals_;
  std::shared_ptr<Camera> camera_;

  // Data.
  std::string global_frame_name_;

  // Publishers.
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;

  // Internal methods
  /**
   * Projects the camera view for pose T_M_C into a semantic segmentation image,
   * depth image and uncertainty image This functions does not need a valid
   * depth Image and fills up the depth image with voxel centers.
   *
   * @param T_M_C Transform from camera to world
   * @param rendered_image Image for which the visible classes are rendered
   * @param uncertainty_image Image for which the uncertainty values are stored
   * @param depth_image Image for which the current depth of the pose is stored
   * @param uncertainty_method Which uncertainty method to use
   * <uncertainty|voxel_entropy|voxel_probability>
   */
  void renderImageForPose(SubmapCollection* submaps,
                          const Transformation& T_M_C,
                          cv::Mat* rendered_image) const;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_CAMERA_RENDERER_H_
