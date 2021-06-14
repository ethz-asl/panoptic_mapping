#include "panoptic_mapping_ros/visualization/planning_visualizer.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace panoptic_mapping {

void PlanningVisualizer::Config::checkParams() const {
  checkParamGT(slice_resolution, 0.f, "slice_resolution");
}

void PlanningVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("visualize_planning_slice", &visualize_planning_slice);
  setupParam("slice_resolution", &slice_resolution);
  setupParam("slice_height", &slice_height);
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
  const Submap& submap_0 =
      *(planning_interface_->getSubmapCollection().begin());
  Point center_M =
      submap_0.getT_M_S() * submap_0.getBoundingVolume().getCenter();
  float x_min = center_M.x();
  float x_max = center_M.x();
  float y_min = center_M.y();
  float y_max = center_M.y();
  for (const auto& submap : planning_interface_->getSubmapCollection()) {
    center_M = submap.getT_M_S() * submap.getBoundingVolume().getCenter();
    const float radius = submap.getBoundingVolume().getRadius();
    x_min = std::min(x_min, center_M.x() - radius);
    x_max = std::max(x_max, center_M.x() + radius);
    y_min = std::min(y_min, center_M.y() - radius);
    y_max = std::max(y_max, center_M.y() + radius);
  }
  size_t x_steps = (x_max - x_min) / config_.slice_resolution;
  size_t y_steps = (y_max - y_min) / config_.slice_resolution;

  // Setup message.
  visualization_msgs::Marker marker;
  marker.header.frame_id = global_frame_name_;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.id = 0;
  marker.scale.x = config_.slice_resolution;
  marker.scale.y = config_.slice_resolution;
  marker.scale.z = config_.slice_resolution;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.points.reserve(x_steps * y_steps);
  marker.colors.reserve(x_steps * y_steps);

  // If requested monitor querry time.
  std::vector<float> times;
  if (config_.verbosity >= 3) {
    times.reserve(x_steps * y_steps);
  }

  // Generate all points.
  for (size_t x = 0; x < x_steps; ++x) {
    for (size_t y = 0; y < y_steps; ++y) {
      Point position(x_min + static_cast<float>(x) * config_.slice_resolution,
                     y_min + static_cast<float>(y) * config_.slice_resolution,
                     config_.slice_height);
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      marker.points.emplace_back(point);
      std_msgs::ColorRGBA color;
      color.a = 0.7;
      PlanningInterface::VoxelState state;
      if (config_.verbosity >= 3) {
        auto t_start = std::chrono::high_resolution_clock::now();
        state = planning_interface_->getVoxelState(position);
        auto t_end = std::chrono::high_resolution_clock::now();
        times.emplace_back(
            static_cast<float>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t_end -
                                                                     t_start)
                    .count()) /
            1000.f);
      } else {
        state = planning_interface_->getVoxelState(position);
      }
      switch (state) {
        case PlanningInterface::VoxelState::kUnknown: {
          color.r = 0.5;
          color.g = 0.5;
          color.b = 0.5;
          break;
        }
        case PlanningInterface::VoxelState::kKnownFree: {
          color.r = 0.3;
          color.g = 0.3;
          color.b = 1.0;
          break;
        }
        case PlanningInterface::VoxelState::kExpectedFree: {
          color.r = 0.7;
          color.g = 0.7;
          color.b = 1.0;
          break;
        }
        case PlanningInterface::VoxelState::kKnownOccupied: {
          color.r = 1.0;
          color.g = 0.3;
          color.b = 0.3;
          break;
        }
        case PlanningInterface::VoxelState::kExpectedOccupied: {
          color.r = 1.0;
          color.g = 0.7;
          color.b = 0.7;
          break;
        }
      }
      marker.colors.emplace_back(color);
    }
  }
  if (config_.verbosity >= 3 && !times.empty()) {
    // Print timings.
    const float mean =
        std::accumulate(times.begin(), times.end(), 0.f) / times.size();
    const float square_sum =
        std::inner_product(times.begin(), times.end(), times.begin(), 0.f);
    const float stddev = std::sqrt(square_sum / times.size() - mean * mean);
    const float max = *std::max_element(times.begin(), times.end());
    LOG(INFO) << "Map lookups based on "
              << planning_interface_->getSubmapCollection().size()
              << " submaps took " << std::fixed << std::setprecision(1) << mean
              << "+/-" << std::fixed << std::setprecision(1) << stddev
              << ", max " << std::fixed << std::setprecision(1) << max << "us.";
  }
  return marker;
}

}  // namespace panoptic_mapping
