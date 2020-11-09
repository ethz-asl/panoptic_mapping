#include "panoptic_mapping_ros/conversions/ros_component_factory.h"

#include <memory>
#include <string>
#include <utility>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>

#include "panoptic_mapping_ros/conversions/ros_params.h"

namespace panoptic_mapping {

std::string ComponentFactoryROS::getType(const ros::NodeHandle& nh) {
  std::string type;
  nh.param("type", type, std::string("type param is not set"));
  return type;
}

std::unique_ptr<IntegratorBase> ComponentFactoryROS::createIntegrator(
    const ros::NodeHandle& nh) {
  std::string type = getType(nh);
  if (type == "naive") {
    return std::make_unique<NaiveIntegrator>(
        getNaiveIntegratorConfigFromRos(nh));
  } else if (type == "projective") {
    return std::make_unique<ProjectiveIntegrator>(
        config_utilities::getConfigFromRos<ProjectiveIntegrator::Config>(nh));
  } else {
    LOG(ERROR) << "Unknown integrator type '" << type << "'.";
    return nullptr;
  }
}

}  // namespace panoptic_mapping
