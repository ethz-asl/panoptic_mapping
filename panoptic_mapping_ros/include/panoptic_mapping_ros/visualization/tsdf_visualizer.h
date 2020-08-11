#ifndef PANOPTIC_MAPPING_SRC_VISUALIZATION_MESH_VISUALIZER_H_
#define PANOPTIC_MAPPING_SRC_VISUALIZATION_MESH_VISUALIZER_H_

#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/common.h"

#include <ros/node_handle.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/MultiMesh.h>
#include <visualization_msgs/MarkerArray.h>

namespace panoptic_mapping {

class TsdfVisualizer {
 public:
  TsdfVisualizer() = default;
  virtual ~TsdfVisualizer() = default;

  // init
  bool setupFromRos(const ros::NodeHandle &nh);

  // meshing
  void setMeshColoringMode(const std::string &coloring_mode) { mesh_coloring_method_ = coloring_mode; }
  bool updatehMesh(Submap *submap, bool update_all_blocks = false);
  void generateMeshMsg(Submap *submap, voxblox_msgs::MultiMesh *output, bool mesh_all_blocks=false);

  // blocks
  void generateBlockMsg(const Submap &submap, visualization_msgs::MarkerArray *output);

  // accessors
  void setMeshColoringMethod(const std::string &method) { mesh_coloring_method_ = method; }

 private:
  voxblox::MeshIntegratorConfig config_;
  std::unique_ptr<voxblox::MeshIntegrator<TsdfVoxel>> mesh_integrator_;

  std::string mesh_coloring_method_;    // 'color' (default), 'normals', 'submap_color'
};

} // namespace panoptic_mapping

#endif //PANOPTIC_MAPPING_SRC_VISUALIZATION_MESH_VISUALIZER_H_
