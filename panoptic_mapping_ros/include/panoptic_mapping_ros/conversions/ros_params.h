#ifndef PANOPTIC_MAPPING_ROS_ROS_PARAMS_H_
#define PANOPTIC_MAPPING_ROS_ROS_PARAMS_H_

#include <memory>
#include <string>

#include <ros/param.h>

#include "panoptic_mapping/integrator/naive_integrator.h"
#include "panoptic_mapping/integrator/projective_integrator.h"
#include "panoptic_mapping/integrator/projective_multi_tsdf_integrator.h"

namespace panoptic_mapping {

// integrator configs
NaiveIntegrator::Config getNaiveIntegratorConfigFromRos(const ros::NodeHandle &nh);

ProjectiveIntegrator::Config getProjectiveIntegratorConfigFromRos(const ros::NodeHandle &nh);

ProjectiveMutliTSDFIntegrator::Config getProjectiveMutliTSDFIntegratorConfigFromRos(const ros::NodeHandle &nh);

std::unique_ptr<IntegratorBase::Config> getTSDFIntegratorConfigFromRos(const ros::NodeHandle &nh,
                                                                       const std::string &type);

} //panoptic_mapping

#endif //PANOPTIC_MAPPING_ROS_ROS_PARAMS_H_
