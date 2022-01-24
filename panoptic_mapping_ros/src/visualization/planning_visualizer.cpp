#include "panoptic_mapping_ros/visualization/planning_visualizer.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <panoptic_mapping/common/globals.h>
#include <std_msgs/Header.h>

namespace panoptic_mapping {

void PlanningVisualizer::Config::checkParams() const {
  checkParamGT(slice_resolution, 0.f, "slice_resolution");
  checkParamGT(turtlebot_map_size, 0.f, "turtlebot_map_size");
  checkParamGT(turtlebot_resolution, 0.f, "turtlebot_resolution");
}

void PlanningVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("visualize_planning_slice", &visualize_planning_slice);
  setupParam("slice_resolution", &slice_resolution);
  setupParam("slice_height", &slice_height);
  setupParam("turtlebot_map_size", &turtlebot_map_size);
  setupParam("turtlebot_resolution", &turtlebot_resolution);
  setupParam("turtlebot_orientation_is_fixed", &turtlebot_orientation_is_fixed);
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
  turtlebot_pub_ =
      nh_.advertise<sensor_msgs::Image>("turtlebot_local_map", 100);
}

void PlanningVisualizer::visualizeAll() {
  visualizePlanningSlice();
  publishTurtlebotMap();
}

void PlanningVisualizer::visualizePlanningSlice() {
  if (config_.visualize_planning_slice && slice_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker msg = generateSliceMsg();
    slice_pub_.publish(msg);
  }
}

void PlanningVisualizer::publishTurtlebotMap() {
  const Transformation T_W_C = globals_->getT_W_C();
  const int extent =
      config_.turtlebot_map_size / config_.turtlebot_resolution;  // px
  const float half_extent = config_.turtlebot_map_size / 2.f;
  cv::Mat result(extent, extent, CV_8UC1, cv::Scalar(2));

  // Get image data from map.
  for (size_t x = 0; x < extent; ++x) {
    for (size_t y = 0; y < extent; ++y) {
      Point position_C(
          static_cast<float>(y) * config_.turtlebot_resolution - half_extent, 0,
          half_extent - static_cast<float>(x) * config_.turtlebot_resolution);
      Point position_W;
      if (config_.turtlebot_orientation_is_fixed) {
        // Only position offset.
        position_W = position_C + T_W_C.getPosition();
      } else {
        // This includes rotation.
        position_W = T_W_C * position_C;
      }
      position_W.z() = config_.slice_height;

      PlanningInterface::VoxelState state =
          planning_interface_->getVoxelState(position_W);

      if (state == PlanningInterface::VoxelState::kKnownFree) {
        result.at<uchar>(x, y) = 0;
      } else if (state == PlanningInterface::VoxelState::kKnownOccupied) {
        result.at<uchar>(x, y) = 100;
      } else {
        result.at<uchar>(x, y) = 200;
      }
    }
  }

  // Publish.
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  turtlebot_pub_.publish(
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1,
                         result)
          .toImageMsg());
}

visualization_msgs::Marker PlanningVisualizer::generateSliceMsg() {
  // Setup the message.
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

  // Compute extent of the current map.
  float x_min = std::numeric_limits<float>::max();
  float x_max = std::numeric_limits<float>::lowest();
  float y_min = std::numeric_limits<float>::max();
  float y_max = std::numeric_limits<float>::lowest();
  bool extent_set = false;
  for (const auto& submap : planning_interface_->getSubmapCollection()) {
    if (submap.getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }
    const Point center_M =
        submap.getT_M_S() * submap.getBoundingVolume().getCenter();
    const float radius = submap.getBoundingVolume().getRadius();
    x_min = std::min(x_min, center_M.x() - radius);
    x_max = std::max(x_max, center_M.x() + radius);
    y_min = std::min(y_min, center_M.y() - radius);
    y_max = std::max(y_max, center_M.y() + radius);
    extent_set = true;
  }
  if (!extent_set) {
    return marker;
  }

  // Compute the resolution.
  size_t x_steps = (x_max - x_min) / config_.slice_resolution;
  size_t y_steps = (y_max - y_min) / config_.slice_resolution;
  if (x_steps <= 0u || y_steps <= 0u) {
    return marker;
  }

  // General properties.
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
      color.a = 0.6;
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
          color.r = 0.7;
          color.g = 0.7;
          color.b = 0.7;
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
        case PlanningInterface::VoxelState::kPersistentOccupied: {
          color.r = 1.0;
          color.g = 0.7;
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
