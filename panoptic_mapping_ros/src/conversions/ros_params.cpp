#include "panoptic_mapping_ros/conversions/ros_params.h"

#include <voxblox_ros/ros_params.h>

namespace panoptic_mapping {

NaivePointcloudIntegrator::Config getNaivePointcloudIntegratorConfigFromRos(const ros::NodeHandle &nh) {
  NaivePointcloudIntegrator::Config config;
  config.voxblox_integrator_config = voxblox::getTsdfIntegratorConfigFromRosParam(nh);
  return config;
}

ProjectiveMutliTSDFIntegrator::Config getProjectiveMutliTSDFIntegratorConfigFromRos(const ros::NodeHandle &nh) {
  ProjectiveMutliTSDFIntegrator::Config config;
  nh.param("horizontal_resolution", config.horizontal_resolution, config.horizontal_resolution);
  nh.param("vertical_resolution", config.vertical_resolution, config.vertical_resolution);
  nh.param("vertical_fov_deg", config.vertical_fov_deg, config.vertical_fov_deg);
  nh.param("min_ray_length_m", config.min_ray_length_m, config.min_ray_length_m);
  nh.param("max_ray_length_m", config.max_ray_length_m, config.max_ray_length_m);
  nh.param("voxel_carving_enabled", config.voxel_carving_enabled, config.voxel_carving_enabled);
  nh.param("voxel_carving_enabled", config.voxel_carving_enabled, config.voxel_carving_enabled);
  nh.param("use_weight_dropoff", config.use_weight_dropoff, config.use_weight_dropoff);
  nh.param("max_weight", config.max_weight, config.max_weight);
  return config;
}

std::unique_ptr<PointcloudIntegratorBase::Config> getPointcloudIntegratorConfigFromRos(const ros::NodeHandle &nh,
                                                                                       const std::string &type) {
  if (type == "naive") {
    return std::make_unique<NaivePointcloudIntegrator::Config>(getNaivePointcloudIntegratorConfigFromRos(nh));
  } else if (type == "projective_multi_tsdf") {
    return std::make_unique<ProjectiveMutliTSDFIntegrator::Config>(getProjectiveMutliTSDFIntegratorConfigFromRos(nh));
  } else {
    LOG(WARNING) << "Unknown pointcloud integrator type '" << type << "' to get config for.";
    return nullptr;
  }
}

} //panoptic_mapping