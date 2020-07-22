#ifndef PANOPTIC_MAPPING_ROS_CONVERSIONS_ROS_PARAMS_H_
#define PANOPTIC_MAPPING_ROS_CONVERSIONS_ROS_PARAMS_H_

#include <memory>
#include <string>

#include <ros/param.h>

#include <panoptic_mapping/integrator/naive_integrator.h>
#include <panoptic_mapping/integrator/projective_integrator.h>
#include <panoptic_mapping/preprocessing/ground_truth_id_tracker.h>

namespace panoptic_mapping {

// integrator configs
NaiveIntegrator::Config getNaiveIntegratorConfigFromRos(
    const ros::NodeHandle& nh);

ProjectiveIntegrator::Config getProjectiveIntegratorConfigFromRos(
    const ros::NodeHandle& nh);

// id tracker configs
GroundTruthIDTracker::Config getGroundTruthIDTrackerConfigFromRos(
    const ros::NodeHandle& nh);

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_CONVERSIONS_ROS_PARAMS_H_
