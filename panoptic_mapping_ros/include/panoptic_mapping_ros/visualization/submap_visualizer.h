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
    std::string mesh_coloring_mode = "color";  // initial mesh coloring mode
    int submap_color_discretization = 20;
    bool visualize_mesh = true;
    bool visualize_tsdf_blocks = true;
    voxblox::MeshIntegratorConfig mesh_integrator_config;

    [[nodiscard]] Config isValid() const;
  };

  // constructors
  explicit SubmapVisualizer(const Config& config,
                            std::shared_ptr<LabelHandler> label_handler);
  virtual ~SubmapVisualizer() = default;

  // coloring modes
  enum class MeshColoringMode {
    kColor = 0,
    kNormals,
    kSubmaps,
    kInstances,
    kClasses
  };

  // coloring mode conversion
  static MeshColoringMode coloringModeFromString(
      const std::string& coloring_mode);
  static std::string coloringModeToString(MeshColoringMode coloring_mode);

  // visualization
  void generateMeshMsgs(SubmapCollection* submaps,
                        std::vector<voxblox_msgs::MultiMesh>* output);
  void generateBlockMsgs(const SubmapCollection& submaps,
                         visualization_msgs::MarkerArray* output) const;

  // interaction
  void reset();
  void setMeshColoringMode(MeshColoringMode coloring_mode);
  void setGlobalFrameName(const std::string& frame_name) {
    global_frame_name_ = frame_name;
  }

 private:
  const Color kUnknownColor = Color(50, 50, 50);

  struct SubmapVisInfo {
    int id = 0;
    bool remesh_everything = false;
    bool republish_everything = false;
    bool was_deleted = false;
    bool change_color = false;
    Color color;
  };

  void updateSubmapMesh(Submap* submap, bool update_all_blocks = false);
  void updateVisInfos(const SubmapCollection& submaps);
  void setSubmapVisColor(const Submap& submap, SubmapVisInfo* info);
  void publishTfTransform(const Transformation& transform,
                          const std::string& parent_frame,
                          const std::string& child_frame);

 private:
  const Config config_;
  MeshColoringMode coloring_mode_;
  std::string global_frame_name_;

  std::shared_ptr<LabelHandler> label_handler_;
  std::unique_ptr<voxblox::MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  std::unordered_map<int, SubmapVisInfo> vis_infos_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_SUBMAP_VISUALIZER_H_
