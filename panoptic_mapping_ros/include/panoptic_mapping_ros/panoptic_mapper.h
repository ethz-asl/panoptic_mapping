#ifndef PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
#define PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/common/globals.h>
#include <panoptic_mapping/integration/tsdf_integrator_base.h>
#include <panoptic_mapping/map/submap.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/map_management/map_manager_base.h>
#include <panoptic_mapping/tools/data_writer_base.h>
#include <panoptic_mapping/tools/planning_interface.h>
#include <panoptic_mapping/tools/thread_safe_submap_collection.h>
#include <panoptic_mapping/tracking/id_tracker_base.h>
#include <panoptic_mapping_msgs/SaveLoadMap.h>
#include <panoptic_mapping_msgs/SetVisualizationMode.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <voxblox/core/esdf_map.h>

#include "panoptic_mapping_ros/input/input_synchronizer.h"
#include "panoptic_mapping_ros/visualization/camera_renderer.h"
#include "panoptic_mapping_ros/visualization/planning_visualizer.h"
#include "panoptic_mapping_ros/visualization/submap_visualizer.h"
#include "panoptic_mapping_ros/visualization/tracking_visualizer.h"

namespace panoptic_mapping {

class PanopticMapper {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 2;

    // Frame name used for the global frame (often mission, world, or odom).
    std::string global_frame_name = "mission";

    // How frequently to perform tasks. Execution period in seconds. Use -1 for
    // every frame, 0 for never.
    float visualization_interval = -1.f;
    float data_logging_interval = 0.f;
    float print_timing_interval = 0.f;

    // How often esdf map should be recalculated.
    float esdf_calculation_interval = 0.5;

    // If true maintain and update the threadsafe submap collection for access.
    bool use_threadsafe_submap_collection = false;

    // Number of threads used for ROS spinning.
    int ros_spinner_threads = std::thread::hardware_concurrency();

    // Frequency in seconds in which the input queue is queried.
    float check_input_interval = 0.01f;

    // If true loaded submaps change states are set to unknown, otherwise to
    // persistent.
    bool load_submaps_conservative = true;

    // If true, finish mapping and shutdown the panoptic mapper when no frames
    // are received for 3 seconds after the first frame was received.
    bool shutdown_when_finished = false;

    // Set this string to automatically save the map to the specified file when
    // shutting down when finished.
    std::string save_map_path_when_finished = "";

    // If true, display units when printing the component configs.
    bool display_config_units = true;

    // If true, indicate the default values when printing component configs.
    bool indicate_default_values = true;

    Config() { setConfigName("PanopticMapper"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Construction.
  PanopticMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~PanopticMapper() = default;

  // ROS callbacks.
  // Timers.
  void publishVisualizationCallback(const ros::TimerEvent&);
  void dataLoggingCallback(const ros::TimerEvent&);
  void printTimingsCallback(const ros::TimerEvent&);
  void inputCallback(const ros::TimerEvent&);
  void esdfTimerCallback(const ros::TimerEvent&);

  // Services.
  bool saveMapCallback(
      panoptic_mapping_msgs::SaveLoadMap::Request& request,     // NOLINT
      panoptic_mapping_msgs::SaveLoadMap::Response& response);  // NOLINT
  bool loadMapCallback(
      panoptic_mapping_msgs::SaveLoadMap::Request& request,     // NOLINT
      panoptic_mapping_msgs::SaveLoadMap::Response& response);  // NOLINT
  bool setVisualizationModeCallback(
      panoptic_mapping_msgs::SetVisualizationMode::Request& request,  // NOLINT
      panoptic_mapping_msgs::SetVisualizationMode::Response&          // NOLINT
          response);
  bool printTimingsCallback(std_srvs::Empty::Request& request,      // NOLINT
                            std_srvs::Empty::Response& response);   // NOLINT
  bool finishMappingCallback(std_srvs::Empty::Request& request,     // NOLINT
                             std_srvs::Empty::Response& response);  // NOLINT
  bool saveVoxelAsPcCallback(
        panoptic_mapping_msgs::SaveLoadMap::Request& request,     // NOLINT
        panoptic_mapping_msgs::SaveLoadMap::Response& response);  // NOLINT

    // Processing.
  // Integrate a set of input images. The input is usually gathered from ROS
  // topics and provided by the InputSynchronizer.
  void processInput(InputData* input);

  // Performs various post-processing actions.
  // NOTE(schmluk): This is currently a preliminary tool to play around with.
  void finishMapping();

  // IO.
  bool saveMap(const std::string& file_path);
  bool loadMap(const std::string& file_path);

  // Utilities.
  // Print all timings (from voxblox::timing) to console.
  void printTimings() const;

  // Update the meshes and publish the all visualizations of the current map.
  void publishVisualization();

  // Label Voxel
  bool labelClassVoxel(const voxblox::GlobalIndex index, int submap_id,
                       int grountdtruth_class);
  // Access.
  const SubmapCollection& getSubmapCollection() const { return *submaps_; }
  const ThreadSafeSubmapCollection& getThreadSafeSubmapCollection() const {
    return *thread_safe_submaps_;
  }
  const PlanningInterface& getPlanningInterface() const {
    return *planning_interface_;
  }
  MapManagerBase* getMapManagerPtr() { return map_manager_.get(); }
  const Config& getConfig() const { return config_; }

  // Esdf map access
  const voxblox::EsdfMap* getEsdfMapPtr() { return esdf_map_.get(); };

 private:
  // Setup.
  void setupMembers();
  void setupCollectionDependentMembers();
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
  ros::ServiceServer finish_mapping_srv_;
  ros::ServiceServer save_pc_srv_;
  ros::Timer visualization_timer_;
  ros::Timer data_logging_timer_;
  ros::Timer print_timing_timer_;
  ros::Timer input_timer_;
  ros::Timer esdf_timer_;
  ros::Publisher changed_esdf_voxel_pub_;
  // Members.
  const Config config_;

  // Map.
  std::shared_ptr<SubmapCollection> submaps_;
  std::shared_ptr<ThreadSafeSubmapCollection> thread_safe_submaps_;

  // Esdf Map
  std::shared_ptr<voxblox::EsdfMap> esdf_map_;

  // Mapping.
  std::unique_ptr<IDTrackerBase> id_tracker_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;
  std::unique_ptr<MapManagerBase> map_manager_;

  std::unique_ptr<voxblox::EsdfIntegrator> esdf_integrator_;

  // Tools.
  std::shared_ptr<Globals> globals_;
  std::unique_ptr<InputSynchronizer> input_synchronizer_;
  std::unique_ptr<DataWriterBase> data_logger_;
  std::shared_ptr<PlanningInterface> planning_interface_;

  // Visualization.
  std::unique_ptr<SubmapVisualizer> submap_visualizer_;
  std::unique_ptr<PlanningVisualizer> planning_visualizer_;
  std::unique_ptr<TrackingVisualizer> tracking_visualizer_;
  std::unique_ptr<CameraRenderer> camera_renderer_;

  // Which processing to perform.
  bool compute_vertex_map_ = false;
  bool compute_validity_image_ = false;

  // Tracking variables.
  ros::WallTime previous_frame_time_ = ros::WallTime::now();
  std::unique_ptr<Timer> frame_timer_;
  ros::Time last_input_;
  bool got_a_frame_ = false;

  // Default namespaces and types for modules are defined here.
  static const std::map<std::string, std::pair<std::string, std::string>>
      default_names_and_types_;
  ros::NodeHandle defaultNh(const std::string& key) const;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
