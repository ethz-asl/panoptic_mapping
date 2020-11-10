#include "panoptic_mapping_ros/panoptic_mapper.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <voxblox/core/color.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox_msgs/MultiMesh.h>

#include <panoptic_mapping/SubmapCollection.pb.h>

#include "panoptic_mapping_ros/conversions/ros_component_factory.h"
#include "panoptic_mapping_ros/conversions/ros_params.h"

namespace panoptic_mapping {

void PanopticMapper::Config::checkParams() const {
  checkParamGT(max_image_queue_length, 0, "max_image_queue_length");
}

void PanopticMapper::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("max_image_queue_length", &max_image_queue_length);
  setupParam("global_frame_name", &global_frame_name);
  setupParam("visualization_interval", &visualization_interval);
  setupParam("change_detection_interval", &change_detection_interval);
}

PanopticMapper::PanopticMapper(const ::ros::NodeHandle& nh,
                               const ::ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      config_(
          config_utilities::getConfigFromRos<PanopticMapper::Config>(nh_private)
              .checkValid()) {
  setupMembers();
  setupRos();
}

void PanopticMapper::setupMembers() {
  // labels
  std::string label_path;
  nh_private_.param("label_path", label_path, std::string(""));
  label_handler_ = std::make_shared<LabelHandler>();
  label_handler_->readLabelsFromFile(label_path);

  // tsdf_integrator
  ros::NodeHandle integrator_nh(nh_private_, "tsdf_integrator");
  tsdf_integrator_ = ComponentFactoryROS::createIntegrator(integrator_nh);

  // visualization
  ros::NodeHandle visualization_nh(nh_private_, "visualization");
  submap_visualizer_ = std::make_unique<SubmapVisualizer>(
      config_utilities::getConfigFromRos<SubmapVisualizer::Config>(
          visualization_nh),
      label_handler_);
  submap_visualizer_->setGlobalFrameName(config_.global_frame_name);

  // id tracking
  ros::NodeHandle id_tracker_nh(nh_private_, "id_tracker");
  id_tracker_ = config_utilities::FactoryRos::create<IDTrackerBase>(
      id_tracker_nh, label_handler_);

  // tsdf registrator
  ros::NodeHandle registrator_nh(nh_private_, "tsdf_registrator");
  tsdf_registrator_ = std::make_unique<TsdfRegistrator>(
      config_utilities::getConfigFromRos<TsdfRegistrator::Config>(
          registrator_nh));
}

void PanopticMapper::setupRos() {
  // Subscribers.
  pointcloud_sub_ = nh_.subscribe("pointcloud_in", 10,
                                  &PanopticMapper::pointcloudCallback, this);
  depth_image_sub_ =
      nh_.subscribe("depth_image_in", config_.max_image_queue_length,
                    &PanopticMapper::depthImageCallback, this);
  color_image_sub_ =
      nh_.subscribe("color_image_in", config_.max_image_queue_length,
                    &PanopticMapper::colorImageCallback, this);
  segmentation_image_sub_ =
      nh_.subscribe("segmentation_image_in", config_.max_image_queue_length,
                    &PanopticMapper::segmentationImageCallback, this);

  // Services.
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &PanopticMapper::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &PanopticMapper::loadMapCallback, this);
  set_visualization_mode_srv_ = nh_private_.advertiseService(
      "set_visualization_mode", &PanopticMapper::setVisualizationModeCallback,
      this);
  set_color_mode_srv_ = nh_private_.advertiseService(
      "set_color_mode", &PanopticMapper::setColorModeCallback, this);

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
}

void PanopticMapper::processImages(
    const sensor_msgs::ImagePtr& depth_img,
    const sensor_msgs::ImagePtr& color_img,
    const sensor_msgs::ImagePtr& segmentation_img) {
  ros::WallTime t0 = ros::WallTime::now();

  // look up transform
  voxblox::Transformation T_M_C;
  if (!tf_transformer_.lookupTransform(config_.global_frame_name,
                                       depth_img->header.frame_id,
                                       depth_img->header.stamp, &T_M_C)) {
    LOG(WARNING) << "Unable to look up transform from '"
                 << depth_img->header.frame_id << " ' to '"
                 << config_.global_frame_name << "', ignoring images.";
    return;
  }

  // read images, segmentation is mutable (copied) for the id tracker
  cv_bridge::CvImageConstPtr depth =
      cv_bridge::toCvShare(depth_img, depth_img->encoding);
  cv_bridge::CvImageConstPtr color =
      cv_bridge::toCvShare(color_img, color_img->encoding);
  cv_bridge::CvImagePtr segmentation =
      cv_bridge::toCvCopy(segmentation_img, segmentation_img->encoding);
  ros::WallTime t1 = ros::WallTime::now();

  // allocate new submaps
  id_tracker_->processImages(&submaps_, T_M_C, depth->image, color->image,
                             &segmentation->image);
  ros::WallTime t2 = ros::WallTime::now();

  // integrate the images
  tsdf_integrator_->processImages(&submaps_, T_M_C, depth->image, color->image,
                                  segmentation->image);
  ros::WallTime t3 = ros::WallTime::now();
  LOG_IF(INFO, config_.verbosity >= 2) << "Integrated images.";
  LOG_IF(INFO, config_.verbosity >= 3)
      << "(id tracking: " << int((t2 - t1).toSec() * 1000)
      << " + integration: " << int((t3 - t2).toSec() * 1000) << " = "
      << int((t3 - t0).toSec() * 1000) << "ms)";
}

void PanopticMapper::pointcloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  ros::WallTime t0 = ros::WallTime::now();

  // look up the transform
  voxblox::Transformation T_S_C;
  if (!tf_transformer_.lookupTransform(config_.global_frame_name,
                                       pointcloud_msg->header.frame_id,
                                       pointcloud_msg->header.stamp, &T_S_C)) {
    LOG(WARNING) << "Unable to look up transform from '"
                 << pointcloud_msg->header.frame_id << " ' to '"
                 << config_.global_frame_name << "', ignoring pointcloud.";
    return;
  }

  // Convert the pointcloud msg into a voxblox::Pointcloud, color, and ID
  // currently just assume prepared pointcloud message with matching fields
  voxblox::Pointcloud pointcloud;
  voxblox::Colors colors;
  std::vector<int> ids;  // IDs reflect the submap ID a point is associated with
  size_t n_points = pointcloud_msg->height * pointcloud_msg->width;
  pointcloud.reserve(n_points);
  colors.reserve(n_points);
  ids.reserve(n_points);

  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_id(*pointcloud_msg, "id");
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointcloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_rgb(*pointcloud_msg,
                                                          "rgb");
  while (iter_id != iter_id.end()) {
    ids.push_back(iter_id[0]);
    pointcloud.push_back(voxblox::Point(iter_x[0], iter_x[1], iter_x[2]));
    colors.push_back(voxblox::Color(iter_rgb[2], iter_rgb[1],
                                    iter_rgb[0]));  // bgr encoding
    ++iter_id;
    ++iter_x;
    ++iter_rgb;
  }
  ros::WallTime t1 = ros::WallTime::now();

  // allocate new submaps
  id_tracker_->processPointcloud(&submaps_, T_S_C, pointcloud, colors, &ids);
  ros::WallTime t2 = ros::WallTime::now();

  // integrate pointcloud
  tsdf_integrator_->processPointcloud(&submaps_, T_S_C, pointcloud, colors,
                                      ids);
  ros::WallTime t3 = ros::WallTime::now();
  LOG_IF(INFO, config_.verbosity >= 2) << "Integrated point cloud.";
  LOG_IF(INFO, config_.verbosity >= 3)
      << "(conversion: " << int((t1 - t0).toSec() * 1000)
      << " + id tracking: " << int((t2 - t1).toSec() * 1000)
      << " + integration: " << int((t3 - t2).toSec() * 1000) << " = "
      << int((t3 - t0).toSec() * 1000) << "ms)";
}

void PanopticMapper::changeDetectionCallback(const ros::TimerEvent&) {
  tsdf_registrator_->checkSubmapCollectionForChange(submaps_);
}

void PanopticMapper::publishVisualizationCallback(const ros::TimerEvent&) {
  publishVisualization();
}

void PanopticMapper::publishVisualization() {
  submap_visualizer_->visualizeAll(&submaps_);
}

void PanopticMapper::depthImageCallback(const sensor_msgs::ImagePtr& msg) {
  // store depth img in queue
  depth_queue_.push_back(msg);
  if (depth_queue_.size() > config_.max_image_queue_length) {
    depth_queue_.pop_front();
  }
  findMatchingMessagesToPublish(msg);
}

void PanopticMapper::colorImageCallback(const sensor_msgs::ImagePtr& msg) {
  // store color img in queue
  color_queue_.push_back(msg);
  if (color_queue_.size() > config_.max_image_queue_length) {
    color_queue_.pop_front();
  }
  findMatchingMessagesToPublish(msg);
}

void PanopticMapper::segmentationImageCallback(
    const sensor_msgs::ImagePtr& msg) {
  // store segmentation img in queue
  segmentation_queue_.push_back(msg);
  if (segmentation_queue_.size() > config_.max_image_queue_length) {
    segmentation_queue_.pop_front();
  }
  findMatchingMessagesToPublish(msg);
}

void PanopticMapper::findMatchingMessagesToPublish(
    const sensor_msgs::ImagePtr& reference_msg) {
  // check whether there exist 3 images with matching timestamp
  std::deque<sensor_msgs::ImagePtr>::iterator depth_it, color_it,
      segmentation_it;

  depth_it =
      std::find_if(depth_queue_.begin(), depth_queue_.end(),
                   [reference_msg](const sensor_msgs::ImagePtr& s) {
                     return s->header.stamp == reference_msg->header.stamp;
                   });
  if (depth_it == depth_queue_.end()) {
    return;
  }
  color_it =
      std::find_if(color_queue_.begin(), color_queue_.end(),
                   [reference_msg](const sensor_msgs::ImagePtr& s) {
                     return s->header.stamp == reference_msg->header.stamp;
                   });
  if (color_it == color_queue_.end()) {
    return;
  }

  segmentation_it =
      std::find_if(segmentation_queue_.begin(), segmentation_queue_.end(),
                   [reference_msg](const sensor_msgs::ImagePtr& s) {
                     return s->header.stamp == reference_msg->header.stamp;
                   });
  if (segmentation_it == segmentation_queue_.end()) {
    return;
  }

  // a matching set was found
  processImages(*depth_it, *color_it, *segmentation_it);
  depth_queue_.erase(depth_it);
  color_queue_.erase(color_it);
  segmentation_queue_.erase(segmentation_it);
}

bool PanopticMapper::setVisualizationModeCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  SubmapVisualizer::VisualizationMode visualization_mode =
      SubmapVisualizer::visualizationModeFromString(request.file_path);
  submap_visualizer_->setVisualizationMode(visualization_mode);
  LOG(INFO) << "Set visualization mode to '"
            << SubmapVisualizer::visualizationModeToString(visualization_mode)
            << "'.";
  return true;
}

bool PanopticMapper::setColorModeCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  SubmapVisualizer::ColorMode color_mode =
      SubmapVisualizer::colorModeFromString(request.file_path);
  submap_visualizer_->setColorMode(color_mode);
  LOG(INFO) << "Set visualization mode to '"
            << SubmapVisualizer::colorModeToString(color_mode) << "'.";
  return true;
}

bool PanopticMapper::saveMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  return saveMap(request.file_path);
}

bool PanopticMapper::loadMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  return loadMap(request.file_path);
}

// Save load functionality was heavily adapted from cblox
bool PanopticMapper::saveMap(const std::string& file_path) {
  CHECK(!file_path.empty());
  std::fstream outfile;
  outfile.open(file_path, std::fstream::out | std::fstream::binary);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << file_path;
    return false;
  }
  // Saving the submap collection header object
  SubmapCollectionProto submap_collection_proto;
  submap_collection_proto.set_num_submaps(submaps_.size());
  if (!voxblox::utils::writeProtoMsgToStream(submap_collection_proto,
                                             &outfile)) {
    LOG(ERROR) << "Could not write submap collection header message.";
    outfile.close();
    return false;
  }
  // Saving the submaps
  int saved_submaps = 0;
  for (const auto& submap : submaps_) {
    bool success = submap->saveToStream(&outfile);
    if (success) {
      saved_submaps++;
    } else {
      LOG(WARNING) << "Failed to save submap with ID '" << submap->getID()
                   << "'.";
    }
  }
  LOG(INFO) << "Successfully saved " << saved_submaps << "/" << submaps_.size()
            << " submaps.";
  outfile.close();
  return true;
}

bool PanopticMapper::loadMap(const std::string& file_path) {
  // Clear the current maps.
  submaps_.clear();

  // Open and check the file.
  std::fstream proto_file;
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
    submap_ptr->finishActivePeriod();
    tsdf_registrator_->computeIsoSurfacePoints(submap_ptr.get());
    submap_ptr->getBoundingVolumePtr()->update();

    // add to the collection
    submaps_.addSubmap(std::move(submap_ptr));
  }
  proto_file.close();

  // Reproduce the mesh and visualization.
  submap_visualizer_->reset();
  submap_visualizer_->visualizeAll(&submaps_);

  LOG(INFO) << "Successfully loaded " << submaps_.size() << "/"
            << submap_collection_proto.num_submaps() << " submaps.";
  return true;
}

}  // namespace panoptic_mapping
