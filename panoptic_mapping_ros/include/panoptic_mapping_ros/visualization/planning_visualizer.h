#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_PLANNING_VISUALIZER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_PLANNING_VISUALIZER_H_

#include <memory>
#include <string>

#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/tools/planning_interface.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

namespace panoptic_mapping {

class PlanningVisualizer {
 public:
  // config
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    bool visualize_planning_slice = true;
    float planning_slice_resolution = 0.1;  // m
    float planning_slice_height = 1.0;      // m
    std::string ros_namespace;

    Config() { setConfigName("PlanningVisualizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void fromRosParam() override;
    void checkParams() const override;
  };

  // Constructors.
  explicit PlanningVisualizer(
      const Config& config,
      std::shared_ptr<const PlanningInterface> planning_interface);
  virtual ~PlanningVisualizer() = default;

  // Visualization message creation.
  visualization_msgs::Marker generateSliceMsg();

  // Publish visualization requests.
  void visualizeAll();
  void visualizePlanningSlice();

  // Interaction.
  void setGlobalFrameName(const std::string& frame_name) {
    global_frame_name_ = frame_name;
  }

 private:
  const Config config_;

  // Members.
  std::shared_ptr<const PlanningInterface> planning_interface_;

  // Data.
  std::string global_frame_name_;

  // Publishers.
  ros::NodeHandle nh_;
  ros::Publisher slice_pub_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_PLANNING_VISUALIZER_H_
