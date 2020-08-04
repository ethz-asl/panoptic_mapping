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

ProjectiveIntegrator::Config getProjectiveIntegratorConfigFromRos(
    const ros::NodeHandle& nh) {
  ProjectiveIntegrator::Config config;
  nh.param("verbosity", config.verbosity, config.verbosity);
  nh.param("width", config.width, config.width);
  nh.param("height", config.height, config.height);
  nh.param("vx", config.vx, config.vx);
  nh.param("vy", config.vy, config.vy);
  nh.param("focal_length", config.focal_length, config.focal_length);
  nh.param("max_range", config.max_range, config.max_range);
  nh.param("min_range", config.min_range, config.min_range);
  nh.param("integration_threads", config.integration_threads,
           config.integration_threads);
  nh.param("interpolation_method", config.interpolation_method,
           config.interpolation_method);
  nh.param("foreign_rays_clear", config.foreign_rays_clear,
           config.foreign_rays_clear);
  nh.param("use_constant_weight", config.use_constant_weight,
           config.use_constant_weight);
  nh.param("use_weight_dropoff", config.use_weight_dropoff,
           config.use_weight_dropoff);
  return config;
}

GroundTruthIDTracker::Config getGroundTruthIDTrackerConfigFromRos(
    const ros::NodeHandle& nh) {
  GroundTruthIDTracker::Config config;
  nh.param("voxels_per_side", config.voxels_per_side, config.voxels_per_side);
  nh.param("instance_voxel_size", config.instance_voxel_size,
           config.instance_voxel_size);
  nh.param("background_voxel_size", config.background_voxel_size,
           config.background_voxel_size);
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

TsdfRegistrator::Config getTsdfRegistatorConfigFromRos(
    const ros::NodeHandle& nh) {
  TsdfRegistrator::Config config;
  return config;
}

}  // namespace panoptic_mapping
