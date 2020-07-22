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

}  // namespace panoptic_mapping
