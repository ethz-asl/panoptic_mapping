#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_CAMERA_RENDERER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_CAMERA_RENDERER_H_

#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/common/globals.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/tools/planning_interface.h>
#include <ros/node_handle.h>
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

  // different sources for rendering
  enum class RenderingSource {
    kId = 0,
    kBlockIndex,
    kScore,
  };
  RenderingSource renderingSourceFromString(
      const std::string& rendering_source);
  std::string renderingSourceToString(const RenderingSource rendering_source);
  const std::map<std::string, RenderingSource> rendering_source_names = {
      {"id", RenderingSource::kId},
      {"blockindex", RenderingSource::kBlockIndex},
      {"score", RenderingSource::kScore}
  };

  // Interaction.
  void setGlobalFrameName(const std::string& frame_name) {
    global_frame_name_ = frame_name;
  }
  /**
   * Projects the camera view for pose T_M_C into a semantic segmentation image,
   * depth image and uncertainty image This functions does not need a valid
   * depth Image and fills up the depth image with voxel centers.
   *
   * @param data_source the data source to be used, needs to be able to convert
   * to RenderingSource
   */
  void renderCameraView(const SubmapCollection* submaps,
                        const tf::StampedTransform transform,
                        const std::string& data_source,
                        cv::Mat& rendered_image);

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
   * @param submaps the submaps to use for rendering
   * @param T_M_C Transform from camera to world
   * @param rendered_image Image for which the visible classes are rendered
   */
  void renderIdImageForPose(const SubmapCollection* submaps,
                            const Transformation& T_M_C,
                            cv::Mat& rendered_image) const;
  void renderScoreImageForPose(const SubmapCollection* submaps,
                               const Transformation& T_M_C,
                               cv::Mat& rendered_image) const;
  void renderBlockIndexImageForPose(const SubmapCollection* submaps,
                                    const Transformation& T_M_C,
                                    cv::Mat& rendered_image) const;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_CAMERA_RENDERER_H_
