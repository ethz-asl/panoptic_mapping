#ifndef PANOPTIC_MAPPING_ROS_ROS_PARAMS_H_
#define PANOPTIC_MAPPING_ROS_ROS_PARAMS_H_

#include <memory>
#include <string>

#include <ros/param.h>

#include "panoptic_mapping/integrator/naive_integrator.h"
#include "panoptic_mapping/integrator/projective_multi_tsdf_integrator.h"

namespace panoptic_mapping {

// integrator configs
NaivePointcloudIntegrator::Config getNaivePointcloudIntegratorConfigFromRos(const ros::NodeHandle &nh);
ProjectiveMutliTSDFIntegrator::Config getProjectiveMutliTSDFIntegratorConfigFromRos(const ros::NodeHandle &nh);
std::unique_ptr<PointcloudIntegratorBase::Config> getPointcloudIntegratorConfigFromRos(const ros::NodeHandle &nh,
                                                                                       const std::string &type);

} //panoptic_mapping

#endif //PANOPTIC_MAPPING_ROS_ROS_PARAMS_H_
