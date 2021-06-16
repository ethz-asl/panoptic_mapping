#include "panoptic_mapping_ros/panoptic_mapper.h"

#include <memory>
#include <sstream>
#include <string>

namespace panoptic_mapping {

void PanopticMapper::Config::checkParams() const {
  checkParamCond(!global_frame_name.empty(),
                 "'global_frame_name' may not be empty.");
}

void PanopticMapper::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("global_frame_name", &global_frame_name);
  setupParam("visualization_interval", &visualization_interval);
  setupParam("data_logging_interval", &data_logging_interval);
  setupParam("print_timing", &print_timing);
}

PanopticMapper::PanopticMapper(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      config_(
          config_utilities::getConfigFromRos<PanopticMapper::Config>(nh_private)
              .checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  setupMembers();
  setupRos();
}

void PanopticMapper::setupMembers() {
  // Map.
  submaps_ = std::make_shared<SubmapCollection>();

  // Camera.
  auto camera = std::make_shared<Camera>(
      config_utilities::getConfigFromRos<Camera::Config>(
          ros::NodeHandle(nh_private_, "camera")));

  // Labels.
  std::string label_path;
  nh_private_.param("label_path", label_path, std::string(""));
  auto label_handler = std::make_shared<LabelHandler>();
  label_handler->readLabelsFromFile(label_path);

  // Globals.
  globals_ = std::make_shared<Globals>(camera, label_handler);

  // ID Tracking.
  id_tracker_ = config_utilities::FactoryRos::create<IDTrackerBase>(
      ros::NodeHandle(nh_private_, "id_tracker"), globals_);

  // Tsdf Integrator.
  tsdf_integrator_ = config_utilities::FactoryRos::create<TsdfIntegratorBase>(
      ros::NodeHandle(nh_private_, "tsdf_integrator"), globals_);

  // Map Manager.
  map_manager_ = std::make_unique<MapManager>(
      config_utilities::getConfigFromRos<MapManager::Config>(
          ros::NodeHandle(nh_private_, "map_management")),
      submaps_);

  // Planning Interface.
  planning_interface_ = std::make_shared<PlanningInterface>(submaps_);

  // Visualization.
  ros::NodeHandle visualization_nh(nh_private_, "visualization");
  // Submaps.
  submap_visualizer_ = std::make_unique<SubmapVisualizer>(
      config_utilities::getConfigFromRos<SubmapVisualizer::Config>(
          ros::NodeHandle(visualization_nh, "submaps")),
      globals_);
  submap_visualizer_->setGlobalFrameName(config_.global_frame_name);

  // Planning.
  planning_visualizer_ = std::make_unique<PlanningVisualizer>(
      config_utilities::getConfigFromRos<PlanningVisualizer::Config>(
          ros::NodeHandle(visualization_nh, "planning")),
      planning_interface_);
  planning_visualizer_->setGlobalFrameName(config_.global_frame_name);

  // Tracking.
  tracking_visualizer_ = std::make_unique<TrackingVisualizer>(
      config_utilities::getConfigFromRos<TrackingVisualizer::Config>(
          ros::NodeHandle(visualization_nh, "tracking")));
  tracking_visualizer_->registerIDTracker(id_tracker_.get());

  // Data Logging.
  data_logger_ = std::make_unique<DataWriter>(
      config_utilities::getConfigFromRos<DataWriter::Config>(
          ros::NodeHandle(nh_private_, "data_writer")));

  // Setup all input topics.
  input_synchronizer_ = std::make_unique<InputSynchronizer>(
      config_utilities::getConfigFromRos<InputSynchronizer::Config>(
          nh_private_),
      nh_);
  input_synchronizer_->requestInputs(id_tracker_->getRequiredInputs());
  input_synchronizer_->requestInputs(tsdf_integrator_->getRequiredInputs());
  input_synchronizer_->setInputCallback(
      [this](InputData* data) { this->processInput(data); });
  input_synchronizer_->advertiseInputTopics();
}

void PanopticMapper::setupRos() {
  // Services.
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &PanopticMapper::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &PanopticMapper::loadMapCallback, this);
  set_visualization_mode_srv_ = nh_private_.advertiseService(
      "set_visualization_mode", &PanopticMapper::setVisualizationModeCallback,
      this);
  print_timings_srv_ = nh_private_.advertiseService(
      "print_timings", &PanopticMapper::printTimingsCallback, this);
  finish_mapping_srv_ = nh_private_.advertiseService(
      "finish_mapping", &PanopticMapper::finishMappingCallback, this);

  // Timers.
  if (config_.visualization_interval > 0.0) {
    visualization_timer_ = nh_private_.createTimer(
        ros::Duration(config_.visualization_interval),
        &PanopticMapper::publishVisualizationCallback, this);
  }
  if (config_.data_logging_interval > 0.0) {
    data_logging_timer_ =
        nh_private_.createTimer(ros::Duration(config_.data_logging_interval),
                                &PanopticMapper::dataLoggingCallback, this);
  }
}

void PanopticMapper::processInput(InputData* input) {
  CHECK_NOTNULL(input);
  Timer timer("input");

  // Compute and store the vertex map.
  Timer vertex_timer("input/compute_vertices");
  input->setVertexMap(
      globals_->camera()->computeVertexMap(input->depthImage()));
  ros::WallTime t0 = ros::WallTime::now();
  vertex_timer.Stop();

  // Track the segmentation images and allocate new submaps.
  Timer id_timer("input/id_tracking");
  id_tracker_->processInput(submaps_.get(), input);
  ros::WallTime t1 = ros::WallTime::now();
  id_timer.Stop();

  // Integrate the images.
  Timer tsdf_timer("input/tsdf_integration");
  tsdf_integrator_->processInput(submaps_.get(), input);
  ros::WallTime t2 = ros::WallTime::now();
  tsdf_timer.Stop();

  // Perform all requested map management actions.
  Timer management_timer("input/map_management");
  map_manager_->tick();
  ros::WallTime t3 = ros::WallTime::now();
  management_timer.Stop();

  // If requested perform visualization and logging.
  ros::TimerEvent event;
  if (config_.visualization_interval < 0.f) {
    publishVisualizationCallback(event);
  }
  if (config_.data_logging_interval < 0.f) {
    dataLoggingCallback(event);
  }
  ros::WallTime t4 = ros::WallTime::now();
  timer.Stop();

  // Logging.
  std::stringstream info;
  info << "Processed input data.";
  if (config_.verbosity >= 3) {
    info << "\n(tracking: " << int((t1 - t0).toSec() * 1000)
         << " + integration: " << int((t2 - t1).toSec() * 1000)
         << " + management: " << int((t3 - t2).toSec() * 1000);
    if (config_.visualization_interval <= 0.f) {
      info << " + visual: " << int((t4 - t3).toSec() * 1000);
    }
    info << " = " << int((t4 - t0).toSec() * 1000) << "ms)";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << info.str();
  LOG_IF(INFO, config_.print_timing) << "\n" << Timing::Print();
}

void PanopticMapper::finishMapping() { map_manager_->finishMapping(); }

void PanopticMapper::publishVisualization() {
  // Update the submaps meshes.
  Timer timer("visualization");
  for (Submap& submap : *submaps_) {
    submap.updateMesh();
  }
  submap_visualizer_->visualizeAll(submaps_.get());
  planning_visualizer_->visualizeAll();
  timer.Stop();
}

bool PanopticMapper::saveMap(const std::string& file_path) {
  bool success = submaps_->saveToFile(file_path);
  LOG_IF(INFO, success) << "Successfully saved " << submaps_->size()
                        << " submaps to '" << file_path << "'.";
  return success;
}

bool PanopticMapper::loadMap(const std::string& file_path) {
  auto loaded_map = std::make_shared<SubmapCollection>();

  // Load the map.
  if (!loaded_map->loadFromFile(file_path, true)) {
    return false;
  }

  // Loaded submaps are 'from the past' so set them to inactive.
  for (Submap& submap : *loaded_map) {
    submap.finishActivePeriod();
  }

  // Set the map.
  submaps_ = loaded_map;

  // Reproduce the mesh and visualization.
  submap_visualizer_->clearMesh();
  submap_visualizer_->visualizeAll(submaps_.get());

  LOG(INFO) << "Successfully loaded " << submaps_->size() << " submaps.";
  return true;
}

void PanopticMapper::dataLoggingCallback(const ros::TimerEvent&) {
  data_logger_->writeData(ros::Time::now().toSec(), *submaps_);
}

void PanopticMapper::publishVisualizationCallback(const ros::TimerEvent&) {
  publishVisualization();
}

bool PanopticMapper::setVisualizationModeCallback(
    panoptic_mapping_msgs::SetVisualizationMode::Request& request,
    panoptic_mapping_msgs::SetVisualizationMode::Response& response) {
  response.visualization_mode_set = false;
  response.color_mode_set = false;
  bool success = true;

  // Set the visualization mode if requested.
  if (!request.visualization_mode.empty()) {
    SubmapVisualizer::VisualizationMode visualization_mode =
        SubmapVisualizer::visualizationModeFromString(
            request.visualization_mode);
    submap_visualizer_->setVisualizationMode(visualization_mode);
    std::string visualization_mode_is =
        SubmapVisualizer::visualizationModeToString(visualization_mode);
    LOG_IF(INFO, config_.verbosity >= 2)
        << "Set visualization mode to '" << visualization_mode_is << "'.";
    response.visualization_mode_set =
        visualization_mode_is == request.visualization_mode;
    if (!response.visualization_mode_set) {
      success = false;
    }
  }

  // Set the color mode if requested.
  if (!request.color_mode.empty()) {
    SubmapVisualizer::ColorMode color_mode =
        SubmapVisualizer::colorModeFromString(request.color_mode);
    submap_visualizer_->setColorMode(color_mode);
    std::string color_mode_is = SubmapVisualizer::colorModeToString(color_mode);
    LOG_IF(INFO, config_.verbosity >= 2)
        << "Set color mode to '" << color_mode_is << "'.";
    response.color_mode_set = color_mode_is == request.color_mode;
    if (!response.color_mode_set) {
      success = false;
    }
  }

  // Republish the visualization.
  submap_visualizer_->visualizeAll(submaps_.get());
  return success;
}

bool PanopticMapper::saveMapCallback(
    panoptic_mapping_msgs::SaveLoadMap::Request& request,
    panoptic_mapping_msgs::SaveLoadMap::Response& response) {
  response.success = saveMap(request.file_path);
  return response.success;
}

bool PanopticMapper::loadMapCallback(
    panoptic_mapping_msgs::SaveLoadMap::Request& request,
    panoptic_mapping_msgs::SaveLoadMap::Response& response) {
  response.success = loadMap(request.file_path);
  return response.success;
}

bool PanopticMapper::printTimingsCallback(std_srvs::Empty::Request& request,
                                          std_srvs::Empty::Response& response) {
  LOG(INFO) << Timing::Print();
  return true;
}

bool PanopticMapper::finishMappingCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  finishMapping();
  return true;
}

}  // namespace panoptic_mapping
