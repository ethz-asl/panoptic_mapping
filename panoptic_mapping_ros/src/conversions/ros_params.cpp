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

}  // namespace panoptic_mapping
