#include "panoptic_mapping_ros/conversions/ros_params.h"

#include <voxblox_ros/ros_params.h>

namespace panoptic_mapping {

NaiveIntegrator::Config getNaiveIntegratorConfigFromRos(const ros::NodeHandle &nh) {
  NaiveIntegrator::Config config;
  config.voxblox_integrator_config = voxblox::getTsdfIntegratorConfigFromRosParam(nh);
  nh.param("voxblox_integrator_type", config.voxblox_integrator_type, config.voxblox_integrator_type);
  return config;
}

ProjectiveMutliTSDFIntegrator::Config getProjectiveMutliTSDFIntegratorConfigFromRos(const ros::NodeHandle &nh) {
  ProjectiveMutliTSDFIntegrator::Config config;
  nh.param("sensor_horizontal_resolution", config.sensor_horizontal_resolution, config.sensor_horizontal_resolution);
  nh.param("sensor_vertical_resolution", config.sensor_vertical_resolution, config.sensor_vertical_resolution);
  nh.param("sensor_vertical_fov_deg", config.sensor_vertical_fov_deg, config.sensor_vertical_fov_deg);
  nh.param("sensor_is_lidar", config.sensor_is_lidar, config.sensor_is_lidar);
  nh.param("min_ray_length_m", config.min_ray_length_m, config.min_ray_length_m);
  nh.param("max_ray_length_m", config.max_ray_length_m, config.max_ray_length_m);
  nh.param("voxel_carving_enabled", config.voxel_carving_enabled, config.voxel_carving_enabled);
  nh.param("sparsity_compensation_factor", config.sparsity_compensation_factor, config.sparsity_compensation_factor);
  nh.param("use_weight_dropoff", config.use_weight_dropoff, config.use_weight_dropoff);
  nh.param("max_weight", config.max_weight, config.max_weight);
  return config;
}

ProjectiveIntegrator::Config getProjectiveIntegratorConfigFromRos(const ros::NodeHandle &nh) {
  ProjectiveIntegrator::Config config;
  return config;
}

std::unique_ptr<IntegratorBase::Config> getTSDFIntegratorConfigFromRos(const ros::NodeHandle &nh,
                                                                       const std::string &type) {
  if (type == "naive") {
    return std::make_unique<NaiveIntegrator::Config>(getNaiveIntegratorConfigFromRos(nh));
  } else if (type == "projective") {
    return std::make_unique<ProjectiveIntegrator::Config>(getProjectiveIntegratorConfigFromRos(nh));
  } else if (type == "projective_multi_tsdf") {
    return std::make_unique<ProjectiveMutliTSDFIntegrator::Config>(getProjectiveMutliTSDFIntegratorConfigFromRos(nh));
  } else {
    LOG(WARNING) << "Unknown pointcloud integrator type '" << type << "' to get config for.";
    return nullptr;
  }
}

} //panoptic_mapping