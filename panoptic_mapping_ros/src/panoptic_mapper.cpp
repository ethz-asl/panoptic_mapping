#include "panoptic_mapping_ros/panoptic_mapper.h"

#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <panoptic_mapping/SubmapCollection.pb.h>

namespace panoptic_mapping {

void PanopticMapper::Config::checkParams() const {
  checkParamCond(!global_frame_name.empty(),
                 "'global_frame_name' may not be empty.");
}

void PanopticMapper::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("global_frame_name", &global_frame_name);
  setupParam("visualization_interval", &visualization_interval);
  setupParam("change_detection_interval", &change_detection_interval);
  setupParam("data_logging_interval", &data_logging_interval);
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

  // Labels.
  std::string label_path;
  nh_private_.param("label_path", label_path, std::string(""));
  label_handler_ = std::make_shared<LabelHandler>();
  label_handler_->readLabelsFromFile(label_path);

  // Id Tracking.
  ros::NodeHandle id_tracker_nh(nh_private_, "id_tracker");
  id_tracker_ = config_utilities::FactoryRos::create<IDTrackerBase>(
      id_tracker_nh, label_handler_);

  // Tsdf Integrator.
  ros::NodeHandle integrator_nh(nh_private_, "tsdf_integrator");
  tsdf_integrator_ =
      config_utilities::FactoryRos::create<IntegratorBase>(integrator_nh);

  // Tsdf Registrator.
  ros::NodeHandle registrator_nh(nh_private_, "tsdf_registrator");
  tsdf_registrator_ = std::make_unique<TsdfRegistrator>(
      config_utilities::getConfigFromRos<TsdfRegistrator::Config>(
          registrator_nh));

  // Planning Interface.
  planning_interface_ = std::make_shared<PlanningInterface>(submaps_);

  // Visualization.
  ros::NodeHandle visualization_nh(nh_private_, "visualization");
  // Submaps.
  submap_visualizer_ = std::make_unique<SubmapVisualizer>(
      config_utilities::getConfigFromRos<SubmapVisualizer::Config>(
          visualization_nh),
      label_handler_);
  submap_visualizer_->setGlobalFrameName(config_.global_frame_name);

  // Planning.
  planning_visualizer_ = std::make_unique<PlanningVisualizer>(
      config_utilities::getConfigFromRos<PlanningVisualizer::Config>(
          visualization_nh),
      planning_interface_);
  planning_visualizer_->setGlobalFrameName(config_.global_frame_name);

  // Tracking.
  tracking_visualizer_ = std::make_unique<TrackingVisualizer>(
      config_utilities::getConfigFromRos<TrackingVisualizer::Config>(
          visualization_nh));
  tracking_visualizer_->registerIDTracker(id_tracker_.get());

  // Data Logging.
  ros::NodeHandle data_nh(nh_private_, "data_writer");
  data_logger_ = std::make_unique<DataWriter>(
      config_utilities::getConfigFromRos<DataWriter::Config>(data_nh));

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

  // Timers.
  if (config_.visualization_interval > 0.0) {
    visualization_timer_ = nh_private_.createTimer(
        ros::Duration(config_.visualization_interval),
        &PanopticMapper::publishVisualizationCallback, this);
  }
  if (config_.change_detection_interval > 0.0) {
    change_detection_timer_ = nh_private_.createTimer(
        ros::Duration(config_.change_detection_interval),
        &PanopticMapper::changeDetectionCallback, this);
  }
  if (config_.data_logging_interval > 0.0) {
    data_logging_timer_ =
        nh_private_.createTimer(ros::Duration(config_.data_logging_interval),
                                &PanopticMapper::dataLoggingCallback, this);
  }
}

void PanopticMapper::processInput(InputData* input) {
  ros::WallTime t0 = ros::WallTime::now();

  // Preprocess the segmentation images and allocate new submaps.
  id_tracker_->processInput(submaps_.get(), input);
  ros::WallTime t1 = ros::WallTime::now();

  // Integrate the images.
  tsdf_integrator_->processInput(submaps_.get(), input);
  ros::WallTime t2 = ros::WallTime::now();

  // If requested perform change detection and visualization.
  ros::TimerEvent event;
  if (config_.change_detection_interval < 0.f) {
    changeDetectionCallback(event);
  }
  ros::WallTime t3 = ros::WallTime::now();
  if (config_.visualization_interval < 0.f) {
    publishVisualizationCallback(event);
  }
  ros::WallTime t4 = ros::WallTime::now();
  if (config_.data_logging_interval < 0.f) {
    dataLoggingCallback(event);
  }

  // Logging.
  std::stringstream info;
  info << "Processed input data.";
  if (config_.verbosity >= 3) {
    info << "\n(tracking: " << int((t1 - t0).toSec() * 1000)
         << " + integration: " << int((t2 - t1).toSec() * 1000);
    if (config_.change_detection_interval <= 0.f) {
      info << " + change: " << int((t3 - t2).toSec() * 1000);
    }
    if (config_.visualization_interval <= 0.f) {
      info << " + visual: " << int((t4 - t3).toSec() * 1000);
    }
    info << " = " << int((t4 - t0).toSec() * 1000) << "ms)";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << info.str();
}

void PanopticMapper::changeDetectionCallback(const ros::TimerEvent&) {
  tsdf_registrator_->checkSubmapCollectionForChange(*submaps_);
}

void PanopticMapper::dataLoggingCallback(const ros::TimerEvent&) {
  data_logger_->writeData(ros::Time::now().toSec(), *submaps_);
}

void PanopticMapper::publishVisualizationCallback(const ros::TimerEvent&) {
  publishVisualization();
}

void PanopticMapper::publishVisualization() {
  submap_visualizer_->visualizeAll(submaps_.get());
  planning_visualizer_->visualizeAll();
}

bool PanopticMapper::setVisualizationModeCallback(
    panoptic_mapping_msgs::SetVisualizationMode::Request& request,
    panoptic_mapping_msgs::SetVisualizationMode::Response& response) {
  response.visualization_mode_set = true;
  response.color_mode_set = true;

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
  }

  return response.visualization_mode_set && response.color_mode_set;
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

// Save load functionality was heavily adapted from cblox.
bool PanopticMapper::saveMap(const std::string& file_path) {
  CHECK(!file_path.empty());
  std::fstream outfile;
  outfile.open(file_path, std::fstream::out | std::fstream::binary);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << file_path;
    return false;
  }
  // Saving the submap collection header object.
  SubmapCollectionProto submap_collection_proto;
  submap_collection_proto.set_num_submaps(submaps_->size());
  if (!voxblox::utils::writeProtoMsgToStream(submap_collection_proto,
                                             &outfile)) {
    LOG(ERROR) << "Could not write submap collection header message.";
    outfile.close();
    return false;
  }
  // Saving the submaps.
  int saved_submaps = 0;
  for (const auto& submap : *submaps_) {
    bool success = submap->saveToStream(&outfile);
    if (success) {
      saved_submaps++;
    } else {
      LOG(WARNING) << "Failed to save submap with ID '" << submap->getID()
                   << "'.";
    }
  }
  LOG(INFO) << "Successfully saved " << saved_submaps << "/" << submaps_->size()
            << " submaps.";
  outfile.close();
  return true;
}

bool PanopticMapper::loadMap(const std::string& file_path) {
  // Clear the current maps.
  submaps_->clear();

  // Open and check the file.
  std::ifstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file '" << file_path << "'.";
    return false;
  }
  // Unused byte offset result.
  uint64_t tmp_byte_offset = 0u;
  SubmapCollectionProto submap_collection_proto;
  if (!voxblox::utils::readProtoMsgFromStream(
          &proto_file, &submap_collection_proto, &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read the protobuf message.";
    return false;
  }

  // Loading each of the submaps.
  for (size_t sub_map_index = 0u;
       sub_map_index < submap_collection_proto.num_submaps(); ++sub_map_index) {
    std::unique_ptr<Submap> submap_ptr =
        Submap::loadFromStream(&proto_file, &tmp_byte_offset);
    if (submap_ptr == nullptr) {
      LOG(ERROR) << "Failed to load submap '" << sub_map_index
                 << "' from stream.";
      return false;
    }

    // Re-compute cached data and set the relevant flags.
    tsdf_registrator_->computeIsoSurfacePoints(submap_ptr.get());
    submap_ptr->finishActivePeriod();
    if (label_handler_->segmentationIdExists(submap_ptr->getInstanceID())) {
      submap_ptr->setName(label_handler_->getName(submap_ptr->getInstanceID()));
    } else if (submap_ptr->getLabel() == PanopticLabel::kFreeSpace) {
      submap_ptr->setName("FreeSpace");
    }

    // Add to the collection.
    submaps_->addSubmap(std::move(submap_ptr));
  }
  proto_file.close();

  // Reproduce the mesh and visualization.
  submap_visualizer_->reset();
  submap_visualizer_->visualizeAll(submaps_.get());

  LOG(INFO) << "Successfully loaded " << submaps_->size() << "/"
            << submap_collection_proto.num_submaps() << " submaps.";
  return true;
}

}  // namespace panoptic_mapping
