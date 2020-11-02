#ifndef PANOPTIC_MAPPING_ROS_CONVERSIONS_ROS_PARAMS_H_
#define PANOPTIC_MAPPING_ROS_CONVERSIONS_ROS_PARAMS_H_

#include <memory>
#include <string>

#include <ros/param.h>

#include <panoptic_mapping/integrator/naive_integrator.h>
#include <panoptic_mapping/integrator/projective_integrator.h>
#include <panoptic_mapping/preprocessing/ground_truth_id_tracker.h>
#include <panoptic_mapping/registration/tsdf_registrator.h>

#include "panoptic_mapping_ros/visualization/submap_visualizer.h"

namespace panoptic_mapping {

// integrator configs
NaiveIntegrator::Config getNaiveIntegratorConfigFromRos(
    const ros::NodeHandle& nh);

// visualizer configs
SubmapVisualizer::Config getSubmapVisualizerConfigFromRos(
    const ros::NodeHandle& nh);

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_CONVERSIONS_ROS_PARAMS_H_
