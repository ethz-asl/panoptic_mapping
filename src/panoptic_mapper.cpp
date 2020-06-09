#include "panoptic_mapping/panoptic_mapper.h"

#include <sstream>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <voxblox_msgs/MultiMesh.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/core/color.h>

#include "panoptic_mapping/SubmapCollection.pb.h"

namespace panoptic_mapping {

PanopticMapper::PanopticMapper(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private) {
  //pointcloud_integrator_(PointcloudIntegrator::getConfigFromRos(ros::NodeHandle(nh_private, "tsdf_integrator"))){

  //params

  // remaining setup
  setupROS();
  setupMembers();
}

void PanopticMapper::setupROS(){
  // Subscribers
  pointcloud_sub_ = nh_.subscribe("pointcloud_in", 10, &PanopticMapper::pointcloudCallback, this);

  // Publishers
  constexpr int pub_queue_size = 100;
  mesh_pub_ = nh_private_.advertise<voxblox_msgs::MultiMesh>("mesh", pub_queue_size, true);
  tsdf_blocks_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("tsdf_blocks", pub_queue_size, true);

  // Services
  save_map_srv_ = nh_private_.advertiseService("save_map", &PanopticMapper::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService("load_map", &PanopticMapper::loadMapCallback, this);
  set_coloring_mode_srv_ =
      nh_private_.advertiseService("set_coloring_mode", &PanopticMapper::setColoringModeCallback, this);

  // Timers
  mesh_timer_ = nh_private_.createTimer(ros::Duration(1.0), &PanopticMapper::publishMeshCallback, this);
}

void PanopticMapper::setupMembers(){
  // visualization
  std::string coloring_mode;
  nh_private_.param("visualization/coloring_mode", coloring_mode, std::string("color"));
  tsdf_visualizer_.setupConfigFromRos(ros::NodeHandle(nh_private_, "visualization"));
  setColoringMode(coloring_mode);

  // labels
  std::string label_path;
  nh_private_.param("label_path", label_path, std::string(""));
  label_handler_.readLabelsFromFile(label_path);
}

void PanopticMapper::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg) {
  // look up the transform
  voxblox::Transformation T_S_C;
  tf_transformer_.lookupTransform("world", "airsim_drone/Depth_cam", pointcloud_msg->header.stamp, &T_S_C);

  // Convert the pointcloud msg into a segmented voxblox::Pointcloud
  std::vector<voxblox::Pointcloud> clouds;
  std::vector<voxblox::Colors> colors;
  std::vector<int> ids;

  // currently just assume prepared pointcloud message with matching fields
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_id(*pointcloud_msg, "id");
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointcloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_rgb(*pointcloud_msg, "rgb");
  int counter = 0;
  while (iter_id != iter_id.end()) {
    int id = iter_id[0];
    auto it = std::find(ids.begin(), ids.end(), id);
    int index = it - ids.begin();
    if (it == ids.end()) {
      ids.emplace_back(id);
      clouds.emplace_back(voxblox::Pointcloud());
      colors.emplace_back(voxblox::Colors());
    }
    clouds.at(index).push_back(voxblox::Point(iter_x[0], iter_x[1], iter_x[2]));
    colors.at(index).push_back(voxblox::Color(iter_rgb[2], iter_rgb[1], iter_rgb[0]));  // bgr encoding
    ++iter_id;
    ++iter_x;
    ++iter_rgb;
    counter++;
  }
  // verbose
  std::stringstream ss;
  if (!ids.empty()) {
    ss << " Found labels are: " << ids[0] << " (" << clouds[0].size() << ")";
    if (ids.size() > 1) {
      for (int i = 1; i < ids.size(); ++i) {
        ss << ", " << ids[i] << " (" << clouds[i].size() << ")";
      }
      ss << ".";
    }
  }
  //VLOG(3) << "Segmented an input pointcloud into " << clouds.size() << " labels, totaling " << counter << " points." << ss.str();

  // integrate pointclouds
  int voxels_per_side = 16;
  double voxel_size;
  ros::WallTime timer_start = ros::WallTime::now();
  for (int i = 0; i < ids.size(); ++i) {
    if (label_handler_.isInstanceClass(ids[i])) {
      voxel_size = 0.03;
    } else {
      voxel_size = 0.08;
      ids[i] = 0;
    }
    if (!submaps_.submapIdExists(ids[i])) {
      // first encounter, allocate a new Submap
      submaps_.addSubmap(Submap(ids[i], voxel_size, voxels_per_side));
      setSubmapColor(&submaps_.getSubmap(ids[i]));
    }
    if (!tsdf_integrator_) {
      tsdf_integrator_.reset(new voxblox::FastTsdfIntegrator(voxblox::TsdfIntegratorBase::Config(),
                                                             submaps_.getSubmap(ids[i]).getTsdfLayerPtr()));
    } else {
      tsdf_integrator_->setLayer(submaps_.getSubmap(ids[i]).getTsdfLayerPtr());
    }
    tsdf_integrator_->integratePointCloud(T_S_C, clouds[i], colors[i]);
    //integrator_.integratePointcloud(clouds[i], colors[i], T_S_C, it->second.tsdf_layer_.get());
  }
  ros::WallDuration time = ros::WallTime::now() - timer_start;
  LOG(INFO) << "Integrated pointcloud (" << time.toSec() << "s)";
}

void PanopticMapper::publishMeshCallback(const ros::TimerEvent &) {
  publishMeshes();

  // tsdf blocks
  for (auto &submap : submaps_) {
    if (submap.getID() == 62) {
      visualization_msgs::MarkerArray markers;
      tsdf_visualizer_.generateBlockMsg(submap, &markers);
      tsdf_blocks_pub_.publish(markers);
    }
  }
}

void PanopticMapper::publishMeshes(bool force_update_all, bool force_mesh_all) {
  for (auto &submap : submaps_) {
    voxblox_msgs::MultiMesh msg;
    tsdf_visualizer_.updatehMesh(&submap, force_update_all);
    tsdf_visualizer_.generateMeshMsg(&submap, &msg, force_mesh_all);
    mesh_pub_.publish(msg);
  }
}

void PanopticMapper::setColoringMode(const std::string &coloring_mode) {
  std::vector<std::string> modes = {"color", "normals", "submaps", "instances"};
  if (std::find(modes.begin(), modes.end(), coloring_mode) == modes.end()) {
    LOG(WARNING) << "Unknown coloring_mode '" << coloring_mode << "', using 'color' instead.";
    coloring_mode_ = "color";
  } else {
    coloring_mode_ = coloring_mode;
  }
  if (coloring_mode_ == "color" || coloring_mode_ == "normals") {
    tsdf_visualizer_.setMeshColoringMode(coloring_mode_);
  } else {
    tsdf_visualizer_.setMeshColoringMode("submap_color");
  }
}

void PanopticMapper::setSubmapColor(Submap *submap) {
  // modes 'color' and 'normals' are handled by the mesher, so no need to set here
  if (coloring_mode_ == "instances") {
    if (label_handler_.segmentationIdExists(submap->getID())) {
      submap->setColor(label_handler_.getColor(submap->getID()));
    } else {
      submap->setColor(Color());    // unknown IDs are black
    }
  } else if (coloring_mode_ == "submaps") {
    constexpr int color_discetization = 30;  // use this many different colors (rainbow)
    float h = float(submap->getID() % color_discetization) / float(color_discetization - 1);
    submap->setColor(voxblox::rainbowColorMap(h));
  }
}

bool PanopticMapper::setColoringModeCallback(voxblox_msgs::FilePath::Request &request,
                                             voxblox_msgs::FilePath::Response &response) {
  setColoringMode(request.file_path);
  for (auto &submap : submaps_) {
    setSubmapColor(&submap);
  }
  publishMeshes(false, true);
  LOG(INFO) << "Set coloring mode to '" << coloring_mode_ << "'.";
  return true;
}

bool PanopticMapper::saveMapCallback(voxblox_msgs::FilePath::Request &request,
                                     voxblox_msgs::FilePath::Response &response) {
  return saveMap(request.file_path);
}
bool PanopticMapper::loadMapCallback(voxblox_msgs::FilePath::Request &request,
                                     voxblox_msgs::FilePath::Response &response) {
  return loadMap(request.file_path);
}

// Save load functionality was heavily adapted from cblox
bool PanopticMapper::saveMap(const std::string &file_path) {
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
  if (!voxblox::utils::writeProtoMsgToStream(submap_collection_proto, &outfile)) {
    LOG(ERROR) << "Could not write submap collection header message.";
    outfile.close();
    return false;
  }
  // Saving the submaps
  int saved_submaps = 0;
  for (const auto &submap : submaps_) {
    bool success = submap.saveToStream(&outfile);
    if (success) {
      saved_submaps++;
    } else {
      LOG(WARNING) << "Failed to save submap with ID '" << submap.getID() << "'.";
    }
  }
  LOG(INFO) << "Successfully saved " << saved_submaps << "/" << submaps_.size() << " submaps.";
  outfile.close();
  return true;
}

bool PanopticMapper::loadMap(const std::string &file_path) {
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
    std::unique_ptr<Submap> submap_ptr = Submap::loadFromStream(&proto_file, &tmp_byte_offset);
    if (submap_ptr == nullptr) {
      LOG(ERROR) << "Failed to load submap '" << sub_map_index << "' from stream.";
      return false;
    }

    // recompute session variables
    setSubmapColor(submap_ptr.get());

    // add to the collection
    submaps_.addSubmap(*submap_ptr);
  }
  proto_file.close();

  // reproduce the mesh and visualization
  publishMeshes(true, true);

  LOG(INFO) << "Successfully loaded " << submaps_.size() << "/" << submap_collection_proto.num_submaps() << " submaps.";
  return true;
}

} // panoptic_mapping