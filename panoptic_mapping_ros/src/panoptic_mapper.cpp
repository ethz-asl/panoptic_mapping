#include "panoptic_mapping_ros/panoptic_mapper.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <voxblox_msgs/MultiMesh.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/core/color.h>

#include <panoptic_mapping/SubmapCollection.pb.h>
#include <panoptic_mapping_ros/conversions/ros_params.h>

#include "panoptic_mapping_ros/conversions/ros_component_factory.h"

namespace panoptic_mapping {

PanopticMapper::PanopticMapper(const ::ros::NodeHandle& nh,
                               const ::ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  setupConfigFromRos();
  setupMembers();
  setupRos();
}

void PanopticMapper::setupConfigFromRos() {
  nh_private_.param("max_image_queue_length", config_.max_image_queue_length,
                    config_.max_image_queue_length);
  nh_private_.param("visualization/coloring_mode", config_.coloring_mode,
                    config_.coloring_mode);
  nh_private_.param("visualization/visualize_mesh", config_.visualize_mesh,
                    config_.visualize_mesh);
  nh_private_.param("visualization/visualize_tsdf_blocks",
                    config_.visualize_tsdf_blocks,
                    config_.visualize_tsdf_blocks);
}

void PanopticMapper::setupRos() {
  // Subscribers
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

  // Publishers
  mesh_pub_ = nh_private_.advertise<voxblox_msgs::MultiMesh>("mesh", 100, true);
  tsdf_blocks_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "tsdf_blocks", 100, true);

  // Services
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &PanopticMapper::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &PanopticMapper::loadMapCallback, this);
  set_coloring_mode_srv_ = nh_private_.advertiseService(
      "set_coloring_mode", &PanopticMapper::setColoringModeCallback, this);

  // Timers
  mesh_timer_ = nh_private_.createTimer(
      ros::Duration(1.0), &PanopticMapper::publishMeshCallback, this);
}

void PanopticMapper::setupMembers() {
  // tsdf_integrator
  ros::NodeHandle integrator_nh(nh_private_, "tsdf_integrator");
  tsdf_integrator_ = ComponentFactoryROS::createIntegrator(integrator_nh);

  // visualization
  ros::NodeHandle visualization_nh(nh_private_, "visualization");
  tsdf_visualizer_.setupFromRos(visualization_nh);
  setColoringMode(config_.coloring_mode);

  // labels
  std::string label_path;
  nh_private_.param("label_path", label_path, std::string(""));
  label_handler_ = std::make_shared<LabelHandler>();
  label_handler_->readLabelsFromFile(label_path);

  // id tracking
  ros::NodeHandle id_tracker_nh(nh_private_, "id_tracker");
  id_tracker_ =
      ComponentFactoryROS::createIDTracker(id_tracker_nh, label_handler_);
}

void PanopticMapper::processImages(
    const sensor_msgs::ImagePtr& depth_img,
    const sensor_msgs::ImagePtr& color_img,
    const sensor_msgs::ImagePtr& segmentation_img) {
  ros::WallTime t0 = ros::WallTime::now();

  // look up transform
  voxblox::Transformation T_M_C;
  if (!tf_transformer_.lookupTransform("world", depth_img->header.frame_id,
                                       depth_img->header.stamp, &T_M_C)) {
    LOG(WARNING) << "Unable to look up transform from '"
                 << depth_img->header.frame_id << " ' to '"
                 << "world"
                 << "', ignoring images.";
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
  VLOG(3) << "Integrated images (id tracking:" << int((t2 - t1).toSec() * 1000)
          << " + integration: " << int((t3 - t2).toSec() * 1000) << " = "
          << int((t3 - t0).toSec() * 1000) << "ms)";
}

void PanopticMapper::pointcloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  ros::WallTime t0 = ros::WallTime::now();

  // look up the transform
  voxblox::Transformation T_S_C;
  if (!tf_transformer_.lookupTransform("world", pointcloud_msg->header.frame_id,
                                       pointcloud_msg->header.stamp, &T_S_C)) {
    LOG(WARNING) << "Unable to look up transform from '"
                 << pointcloud_msg->header.frame_id << " ' to '"
                 << "world"
                 << "', ignoring pointcloud.";
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
  VLOG(3) << "Integrated point cloud (conversion: "
          << int((t1 - t0).toSec() * 1000)
          << " + id tracking:" << int((t2 - t1).toSec() * 1000)
          << " + integration: " << int((t3 - t2).toSec() * 1000) << " = "
          << int((t3 - t0).toSec() * 1000) << "ms)";
}

void PanopticMapper::publishMeshCallback(const ros::TimerEvent&) {
  // update meshes
  if (config_.visualize_mesh) {
    publishMeshes();
  }

  // visualize tsdf blocks
  if (config_.visualize_tsdf_blocks) {
    for (auto& submap : submaps_) {
      // if (label_handler_.isBackgroundClass(submap.getID())) { continue; }
      // if (submap.getID() < 62) { continue; }
      visualization_msgs::MarkerArray markers;
      tsdf_visualizer_.generateBlockMsg(*submap, &markers);
      tsdf_blocks_pub_.publish(markers);
    }
  }
}

void PanopticMapper::publishMeshes(bool force_update_all, bool force_mesh_all) {
  std::vector<int> ids;
  for (auto& submap : submaps_) {
    voxblox_msgs::MultiMesh msg;
    tsdf_visualizer_.updatehMesh(submap.get(), force_update_all);
    tsdf_visualizer_.generateMeshMsg(submap.get(), &msg, force_mesh_all);
    mesh_pub_.publish(msg);
  }

  // test
  for (const auto& id : ids) {
    submaps_.removeSubmap(id);
  }
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

void PanopticMapper::setColoringMode(const std::string& coloring_mode) {
  std::vector<std::string> modes = {"color", "normals", "submaps", "instances"};
  if (std::find(modes.begin(), modes.end(), coloring_mode) == modes.end()) {
    LOG(WARNING) << "Unknown coloring_mode '" << coloring_mode
                 << "', using 'color' instead.";
    config_.coloring_mode = "color";
  } else {
    config_.coloring_mode = coloring_mode;
  }
  if (config_.coloring_mode == "color" || config_.coloring_mode == "normals") {
    tsdf_visualizer_.setMeshColoringMode(config_.coloring_mode);
  } else {
    tsdf_visualizer_.setMeshColoringMode("submap_color");
  }
}

void PanopticMapper::setSubmapColor(Submap* submap) {
  // modes 'color' and 'normals' are handled by the mesher, so no need to set
  if (config_.coloring_mode == "instances") {
    if (label_handler_->segmentationIdExists(submap->getID())) {
      submap->setColor(label_handler_->getColor(submap->getID()));
    } else {
      submap->setColor(Color());    // unknown IDs are black
    }
  } else if (config_.coloring_mode == "submaps") {
    constexpr int color_discetization =
        30;  // use this many different colors (rainbow)
    float h = static_cast<float>(submap->getID() % color_discetization) /
              static_cast<float>(color_discetization - 1);
    submap->setColor(voxblox::rainbowColorMap(h));
  }
}

bool PanopticMapper::setColoringModeCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  setColoringMode(request.file_path);
  for (auto& submap : submaps_) {
    setSubmapColor(submap.get());
  }
  publishMeshes(false, true);
  LOG(INFO) << "Set coloring mode to '" << config_.coloring_mode << "'.";
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
  // Clear the current maps
  submaps_.clear();

  // Open and check the file
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

  // Loading each of the submaps
  for (size_t sub_map_index = 0u;
       sub_map_index < submap_collection_proto.num_submaps(); ++sub_map_index) {
    std::unique_ptr<Submap> submap_ptr =
        Submap::loadFromStream(&proto_file, &tmp_byte_offset);
    if (submap_ptr == nullptr) {
      LOG(ERROR) << "Failed to load submap '" << sub_map_index
                 << "' from stream.";
      return false;
    }

    // recompute session variables
    setSubmapColor(submap_ptr.get());

    // add to the collection
    submaps_.addSubmap(std::move(submap_ptr));
  }
  proto_file.close();

  // reproduce the mesh and visualization
  publishMeshes(true, true);

  LOG(INFO) << "Successfully loaded " << submaps_.size() << "/"
            << submap_collection_proto.num_submaps() << " submaps.";
  return true;
}

}  // namespace panoptic_mapping
