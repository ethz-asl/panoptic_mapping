#include "panoptic_mapping_ros/visualization/planning_visualizer.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace panoptic_mapping {

void PlanningVisualizer::Config::checkParams() const {
  checkParamGT(planning_slice_resolution, 0.f, "planning_slice_resolution");
}

void PlanningVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("visualize_planning_slice", &visualize_planning_slice);
  setupParam("planning_slice_resolution", &planning_slice_resolution);
  setupParam("planning_slice_height", &planning_slice_height);
}

void PlanningVisualizer::Config::fromRosParam() {
  ros_namespace = rosParamNameSpace();
}

PlanningVisualizer::PlanningVisualizer(
    const Config& config,
    std::shared_ptr<const PlanningInterface> planning_interface)
    : config_(config.checkValid()),
      planning_interface_(std::move(planning_interface)),
      global_frame_name_("mission") {
  // Print config after setting up the modes.
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Setup publishers.
  nh_ = ros::NodeHandle(config_.ros_namespace);
  if (config_.visualize_planning_slice) {
    slice_pub_ =
        nh_.advertise<visualization_msgs::Marker>("planning_slice", 100);
  }
}

void PlanningVisualizer::visualizeAll() { visualizePlanningSlice(); }

void PlanningVisualizer::visualizePlanningSlice() {
  if (config_.visualize_planning_slice && slice_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker msg = generateSliceMsg();
    slice_pub_.publish(msg);
  }
}

visualization_msgs::Marker PlanningVisualizer::generateSliceMsg() {
  // Compute extent of the current map.
  float x_min = std::numeric_limits<float>::infinity();
  float x_max = -std::numeric_limits<float>::infinity();
  float y_min = std::numeric_limits<float>::infinity();
  float y_max = -std::numeric_limits<float>::infinity();
}

}  // namespace panoptic_mapping
