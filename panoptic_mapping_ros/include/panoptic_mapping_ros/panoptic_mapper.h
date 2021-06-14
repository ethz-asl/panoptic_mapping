#ifndef PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
#define PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/camera.h>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/common/globals.h>
#include <panoptic_mapping/common/label_handler.h>
#include <panoptic_mapping/integration/tsdf_integrator_base.h>
#include <panoptic_mapping/map/submap.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/map_management/map_manager.h>
#include <panoptic_mapping/tools/data_writer.h>
#include <panoptic_mapping/tools/planning_interface.h>
#include <panoptic_mapping/tracking/id_tracker_base.h>
#include <panoptic_mapping_msgs/SaveLoadMap.h>
#include <panoptic_mapping_msgs/SetVisualizationMode.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "panoptic_mapping_ros/input/input_synchronizer.h"
#include "panoptic_mapping_ros/visualization/planning_visualizer.h"
#include "panoptic_mapping_ros/visualization/submap_visualizer.h"
#include "panoptic_mapping_ros/visualization/tracking_visualizer.h"

namespace panoptic_mapping {

class PanopticMapper {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 2;
    std::string global_frame_name = "mission";
    double visualization_interval = -1.0;    // s, use -1 for always, 0 never.
    double data_logging_interval = 0.0;      // s, use -1 for always, 0 never.
    bool print_timing = false;               // Print timings after every frame.

    Config() { setConfigName("PanopticMapper"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  PanopticMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~PanopticMapper() = default;

  // ROS callbacks.
  void publishVisualizationCallback(const ros::TimerEvent&);
  void dataLoggingCallback(const ros::TimerEvent&);
  bool saveMapCallback(
      panoptic_mapping_msgs::SaveLoadMap::Request& request,     // NOLINT
      panoptic_mapping_msgs::SaveLoadMap::Response& response);  // NOLINT
  bool loadMapCallback(
      panoptic_mapping_msgs::SaveLoadMap::Request& request,     // NOLINT
      panoptic_mapping_msgs::SaveLoadMap::Response& response);  // NOLINT
  bool setVisualizationModeCallback(
      panoptic_mapping_msgs::SetVisualizationMode::Request& request,  // NOLINT
      panoptic_mapping_msgs::SetVisualizationMode::Response&
          response);                                               // NOLINT
  bool printTimingsCallback(std_srvs::Empty::Request& request,     // NOLINT
                            std_srvs::Empty::Response& response);  // NOLINT

  // Processing.
  void processInput(InputData* input);

  // IO.
  bool saveMap(const std::string& file_path);
  bool loadMap(const std::string& file_path);

  // Visualization.
  void publishVisualization();

  // Access.
  const SubmapCollection& getSubmapCollection() const { return *submaps_; }
  const PlanningInterface& getPlanningInterface() const {
    return *planning_interface_;
  }
  MapManager& getMapManager() { return *map_manager_; }

 private:
  // Setup.
  void setupMembers();
  void setupRos();

 private:
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers, Publishers, Services, Timers.
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer set_visualization_mode_srv_;
  ros::ServiceServer set_color_mode_srv_;
  ros::ServiceServer print_timings_srv_;
  ros::Timer visualization_timer_;
  ros::Timer data_logging_timer_;

  // Members.
  const Config config_;

  // Map.
  std::shared_ptr<SubmapCollection> submaps_;

  // Mapping.
  std::unique_ptr<IDTrackerBase> id_tracker_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;
  std::unique_ptr<MapManager> map_manager_;

  // Tools.
  std::shared_ptr<Globals> globals_;
  std::unique_ptr<InputSynchronizer> input_synchronizer_;
  std::unique_ptr<DataWriter> data_logger_;
  std::shared_ptr<PlanningInterface> planning_interface_;

  // Visualization.
  std::unique_ptr<SubmapVisualizer> submap_visualizer_;
  std::unique_ptr<PlanningVisualizer> planning_visualizer_;
  std::unique_ptr<TrackingVisualizer> tracking_visualizer_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
