#include "panoptic_mapping_ros/conversions/ros_component_factory.h"

#include <memory>
#include <string>

#include <voxblox_ros/ros_params.h>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/integrator/naive_integrator.h>
#include <panoptic_mapping/integrator/projective_integrator.h>

namespace panoptic_mapping {

std::unique_ptr<IntegratorBase> ComponentFactoryROS::createIntegrator(
    const ros::NodeHandle& nh) {
  std::string type;
  nh.param("type", type, std::string("type param is not set"));
  if (type == "naive") {
    // Specific implementation for using the standard voxblox integrator.
    NaiveIntegrator::Config config;
    config.voxblox_integrator_config =
        voxblox::getTsdfIntegratorConfigFromRosParam(nh);
    nh.param("voxblox_integrator_type", config.voxblox_integrator_type,
             config.voxblox_integrator_type);
    return std::make_unique<NaiveIntegrator>(config);
  } else if (type == "projective") {
    return std::make_unique<ProjectiveIntegrator>(
        config_utilities::getConfigFromRos<ProjectiveIntegrator::Config>(nh));
  } else {
    LOG(ERROR) << "Unknown integrator type '" << type << "'.";
    return nullptr;
  }
}

}  // namespace panoptic_mapping
