#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_SUBMAP_VISUALIZER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_SUBMAP_VISUALIZER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/node_handle.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/MultiMesh.h>

#include <panoptic_mapping/core/common.h>
#include <panoptic_mapping/core/submap_collection.h>
#include <panoptic_mapping/preprocessing/label_handler.h>

namespace panoptic_mapping {

class SubmapVisualizer {
 public:
  // config
  struct Config {
    std::string visualization_mode = "color";  // initial visualization mode
    int submap_color_discretization = 20;
    bool visualize_mesh = true;
    bool visualize_tsdf_blocks = true;
    voxblox::MeshIntegratorConfig mesh_integrator_config;  // If use_color is
    // false the visualization mode 'color' won't work.

    [[nodiscard]] Config isValid() const;
  };

  // constructors
  explicit SubmapVisualizer(const Config& config,
                            std::shared_ptr<LabelHandler> label_handler);
  virtual ~SubmapVisualizer() = default;

  // visualization modes
  enum class VisualizationMode {
    kColor = 0,
    kNormals,
    kSubmaps,
    kInstances,
    kClasses,
    kChange
  };

  // Visualization mode conversion.
  static VisualizationMode visualizationModeFromString(
      const std::string& visualization_mode);
  static std::string visualizationModeToString(
      VisualizationMode visualization_mode);

  // visualization
  void generateMeshMsgs(SubmapCollection* submaps,
                        std::vector<voxblox_msgs::MultiMesh>* output);
  void generateBlockMsgs(const SubmapCollection& submaps,
                         visualization_msgs::MarkerArray* output) const;

  // interaction
  void reset();
  void setVisualizationMode(VisualizationMode visualization_mode);
  void setGlobalFrameName(const std::string& frame_name) {
    global_frame_name_ = frame_name;
  }

 private:
  const Color kUnknownColor = Color(50, 50, 50);

  struct SubmapVisInfo {
    // General.
    int id = 0;
    bool remesh_everything = false;
    bool republish_everything = false;
    bool was_deleted = false;
    bool change_color = false;
    Color color;
    uint8_t alpha = 255;

    // Tracking: kChange
    int was_matched = 0;  // 0-init, 1-no, 2-yes
  };

  void updateSubmapMesh(Submap* submap, bool update_all_blocks = false);
  void updateVisInfos(const SubmapCollection& submaps);
  void setSubmapVisColor(const Submap& submap, SubmapVisInfo* info);
  void publishTfTransform(const Transformation& transform,
                          const std::string& parent_frame,
                          const std::string& child_frame);

 private:
  const Config config_;
  VisualizationMode visualization_mode_;
  std::string global_frame_name_;

  std::shared_ptr<LabelHandler> label_handler_;
  std::unique_ptr<voxblox::MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::unordered_map<int, SubmapVisInfo> vis_infos_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_SUBMAP_VISUALIZER_H_
