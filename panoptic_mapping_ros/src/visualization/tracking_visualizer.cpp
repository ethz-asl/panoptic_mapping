#include "panoptic_mapping_ros/visualization/tracking_visualizer.h"

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

namespace panoptic_mapping {

void TrackingVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("visualize_tracking", &visualize_tracking);
}

void TrackingVisualizer::Config::fromRosParam() {
  ros_namespace = rosParamNameSpace();
}

TrackingVisualizer::TrackingVisualizer(const Config& config)
    : config_(config.checkValid()) {
  // Print config after setting up the modes.

  // Setup nodehandle.
  nh_ = ros::NodeHandle(config_.ros_namespace);
}

void TrackingVisualizer::registerIDTracker(IDTrackerBase* tracker) {
  if (config_.visualize_tracking) {
    CHECK_NOTNULL(tracker);
    tracker->setVisualizationCallback(
        [this](const cv::Mat& image, const std::string& name) {
          publishImage(image, name);
        });
  }
}

void TrackingVisualizer::publishImage(const cv::Mat& image,
                                      const std::string& name) {
  auto it = publishers_.find(name);
  if (it == publishers_.end()) {
    // Advertise a new topic if there is no publisher for the given name.
    it = publishers_.emplace(name, nh_.advertise<sensor_msgs::Image>(name, 100))
             .first;
  }

  // Publish the image, expected as BGR8.
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  it->second.publish(
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image)
          .toImageMsg());
}

}  // namespace panoptic_mapping
