#include "panoptic_mapping_ros/conversions/ros_params.h"

#include <voxblox_ros/ros_params.h>

namespace panoptic_mapping {

NaiveIntegrator::Config getNaiveIntegratorConfigFromRos(
    const ros::NodeHandle& nh) {
  NaiveIntegrator::Config config;
  config.voxblox_integrator_config =
      voxblox::getTsdfIntegratorConfigFromRosParam(nh);
  nh.param("voxblox_integrator_type", config.voxblox_integrator_type,
           config.voxblox_integrator_type);
  return config;
}

SubmapVisualizer::Config getSubmapVisualizerConfigFromRos(
    const ros::NodeHandle& nh) {
  SubmapVisualizer::Config config;
  nh.param("visualization_mode", config.visualization_mode,
           config.visualization_mode);
  nh.param("visualize_mesh", config.visualize_mesh, config.visualize_mesh);
  nh.param("visualize_tsdf_blocks", config.visualize_tsdf_blocks,
           config.visualize_tsdf_blocks);
  nh.param("submap_color_discretization", config.submap_color_discretization,
           config.submap_color_discretization);
  config.mesh_integrator_config =
      voxblox::getMeshIntegratorConfigFromRosParam(nh);
  return config;
}

}  // namespace panoptic_mapping
