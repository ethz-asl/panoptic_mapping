#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_TRACKING_VISUALIZER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_TRACKING_VISUALIZER_H_

#include <string>
#include <unordered_map>

#include <ros/node_handle.h>

#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/preprocessing/id_tracker_base.h>
#include <panoptic_mapping/3rd_party/config_utilities.hpp>

namespace panoptic_mapping {

class TrackingVisualizer {
 public:
  // config
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    bool visualize_tracking = true;
    std::string ros_namespace;

    Config() { setConfigName("TrackingVisualizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void fromRosParam() override;
  };

  // Constructors.
  explicit TrackingVisualizer(const Config& config);
  virtual ~TrackingVisualizer() = default;

  // Setup.
  void registerIDTracker(IDTrackerBase* tracker);

  // Publish visualization requests.
  void publishImage(const cv::Mat& image, const std::string& name);

 private:
  const Config config_;

  // Publishers.
  ros::NodeHandle nh_;
  std::unordered_map<std::string, ros::Publisher> publishers_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_TRACKING_VISUALIZER_H_
