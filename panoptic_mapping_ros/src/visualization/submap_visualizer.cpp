#include "panoptic_mapping_ros/visualization/submap_visualizer.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <minkindr_conversions/kindr_msg.h>
#include <ros/time.h>
#include <voxblox_ros/ptcloud_vis.h>

namespace panoptic_mapping {

const Color SubmapVisualizer::kUnknownColor_(50, 50, 50);

void SubmapVisualizer::Config::checkParams() const {
  checkParamGT(submap_color_discretization, 0, "submap_color_discretization");
  // NOTE(schmluk): if the visualization or color mode is not valid it will be
  // defaulted to 'all' or 'color' and a warning will be raised.
}

void SubmapVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("visualization_mode", &visualization_mode);
  setupParam("color_mode", &color_mode);
  setupParam("submap_color_discretization", &submap_color_discretization);
  setupParam("visualize_mesh", &visualize_mesh);
  setupParam("visualize_tsdf_blocks", &visualize_tsdf_blocks);
  setupParam("visualize_free_space", &visualize_free_space);
  setupParam("visualize_bounding_volumes", &visualize_bounding_volumes);
  setupParam("include_free_space", &include_free_space);
}

void SubmapVisualizer::Config::printFields() const {
  printField("ros_namespace", ros_namespace);
}

void SubmapVisualizer::Config::fromRosParam() {
  ros_namespace = rosParamNameSpace();
}

SubmapVisualizer::SubmapVisualizer(const Config& config,
                                   std::shared_ptr<LabelHandler> label_handler)
    : config_(config.checkValid()), label_handler_(std::move(label_handler)) {
  visualization_mode_ = visualizationModeFromString(config_.visualization_mode);
  color_mode_ = colorModeFromString(config_.color_mode);
  id_color_map_.setItemsPerRevolution(config_.submap_color_discretization);
  // Print config after setting up the modes.
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Setup publishers.
  nh_ = ros::NodeHandle(config_.ros_namespace);
  if (config_.visualize_free_space) {
    freespace_pub_ =
        nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("free_space_tsdf", 100);
  }
  if (config_.visualize_mesh) {
    mesh_pub_ = nh_.advertise<voxblox_msgs::MultiMesh>("mesh", 1000);
  }
  if (config_.visualize_tsdf_blocks) {
    tsdf_blocks_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("tsdf_blocks", 100);
  }
  if (config_.visualize_bounding_volumes) {
    bounding_volume_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("bounding_volumes", 100);
  }
}

void SubmapVisualizer::reset() {
  // Erase all current tracking / cached data.
  vis_infos_.clear();
  previous_submaps_ = nullptr;
}

void SubmapVisualizer::clearMesh() {
  // Clear the current mesh from the rviz plugin.
  // NOTE(schmluk): Other visuals could also be cleared but since they are
  // non-incremental they will anyways be overwritten.
  if (config_.visualize_mesh && mesh_pub_.getNumSubscribers() > 0) {
    for (auto& info : vis_infos_) {
      voxblox_msgs::MultiMesh msg;
      msg.header.stamp = ros::Time::now();
      msg.name_space = std::to_string(info.second.id);
      mesh_pub_.publish(msg);
    }
  }
}

void SubmapVisualizer::visualizeAll(const SubmapCollection& submaps) {
  publishTfTransforms(submaps);
  updateVisInfos(submaps);
  vis_infos_are_updated_ = true;  // Prevent repeated updates.
  visualizeMeshes(submaps);
  visualizeTsdfBlocks(submaps);
  visualizeFreeSpace(submaps);
  visualizeBoundingVolume(submaps);
  vis_infos_are_updated_ = false;
}

void SubmapVisualizer::visualizeMeshes(const SubmapCollection& submaps) {
  if (config_.visualize_mesh && mesh_pub_.getNumSubscribers() > 0) {
    std::vector<voxblox_msgs::MultiMesh> msgs = generateMeshMsgs(submaps);
    for (auto& msg : msgs) {
      mesh_pub_.publish(msg);
    }
  }
}

void SubmapVisualizer::visualizeTsdfBlocks(const SubmapCollection& submaps) {
  if (config_.visualize_tsdf_blocks &&
      tsdf_blocks_pub_.getNumSubscribers() > 0) {
    visualization_msgs::MarkerArray markers = generateBlockMsgs(submaps);
    tsdf_blocks_pub_.publish(markers);
  }
}

void SubmapVisualizer::visualizeFreeSpace(const SubmapCollection& submaps) {
  if (config_.visualize_free_space && freespace_pub_.getNumSubscribers() > 0) {
    pcl::PointCloud<pcl::PointXYZI> msg = generateFreeSpaceMsg(submaps);
    msg.header.frame_id = global_frame_name_;
    freespace_pub_.publish(msg);
  }
}

void SubmapVisualizer::visualizeBoundingVolume(
    const SubmapCollection& submaps) {
  if (config_.visualize_bounding_volumes &&
      bounding_volume_pub_.getNumSubscribers() > 0) {
    visualization_msgs::MarkerArray markers =
        generateBoundingVolumeMsgs(submaps);
    bounding_volume_pub_.publish(markers);
  }
}

std::vector<voxblox_msgs::MultiMesh> SubmapVisualizer::generateMeshMsgs(
    const SubmapCollection& submaps) {
  std::vector<voxblox_msgs::MultiMesh> result;

  // Update the visualization infos.
  if (!vis_infos_are_updated_) {
    updateVisInfos(submaps);
  }

  // Process all submaps based on their visualization info.
  for (const auto& submap : submaps) {
    if (submap->getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }
    // Find the corresponding info.
    auto it = vis_infos_.find(submap->getID());
    if (it == vis_infos_.end()) {
      LOG(WARNING) << "Tried to visualize submap " << submap->getID()
                   << " without existing SubmapVisInfo.";
      continue;
    }
    SubmapVisInfo& info = it->second;

    // Setup message.
    voxblox_msgs::MultiMesh msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = submap->getFrameName();
    msg.name_space = std::to_string(info.id);

    // If the submap was deleted we send an empty message to delete the visual.
    if (info.was_deleted) {
      result.push_back(msg);
      vis_infos_.erase(it);
      continue;
    }

    // Mark the whole mesh for re-publishing if requested.
    if (info.republish_everything) {
      voxblox::BlockIndexList mesh_indices;
      submap->getMeshLayer().getAllAllocatedMeshes(&mesh_indices);
      for (const auto& block_index : mesh_indices) {
        submap->getMeshLayerPtr()->getMeshPtrByIndex(block_index)->updated =
            true;
      }
      info.republish_everything = false;
    }

    // Set the voxblox internal color mode. Gray will be used for overwriting.
    voxblox::ColorMode color_mode_voxblox = voxblox::ColorMode::kGray;
    if (color_mode_ == ColorMode::kColor) {
      color_mode_voxblox = voxblox::ColorMode::kColor;
    } else if (color_mode_ == ColorMode::kNormals) {
      color_mode_voxblox = voxblox::ColorMode::kNormals;
    }
    voxblox::generateVoxbloxMeshMsg(submap->getMeshLayerPtr(),
                                    color_mode_voxblox, &msg.mesh);

    if (msg.mesh.mesh_blocks.empty()) {
      // Nothing changed, don't send an empty msg which would reset the mesh.
      continue;
    }

    // Apply the submap color if necessary.
    if (color_mode_voxblox == voxblox::ColorMode::kGray) {
      for (auto& mesh_block : msg.mesh.mesh_blocks) {
        for (auto& r : mesh_block.r) {
          r = info.color.r;
        }
        for (auto& g : mesh_block.g) {
          g = info.color.g;
        }
        for (auto& b : mesh_block.b) {
          b = info.color.b;
        }
      }
    }

    // Set alpha values.
    msg.alpha = info.alpha * 255.f;
    result.emplace_back(std::move(msg));
  }
  return result;
}

visualization_msgs::MarkerArray SubmapVisualizer::generateBlockMsgs(
    const SubmapCollection& submaps) {
  visualization_msgs::MarkerArray result;
  // Update the visualization infos.
  if (!vis_infos_are_updated_) {
    updateVisInfos(submaps);
  }

  for (const auto& submap : submaps) {
    if (submap->getLabel() == PanopticLabel::kFreeSpace &&
        !config_.include_free_space) {
      continue;
    }

    // Setup submap.
    voxblox::BlockIndexList blocks;
    submap->getTsdfLayer().getAllAllocatedBlocks(&blocks);
    float block_size =
        submap->getTsdfLayer().voxel_size() *
        static_cast<float>(submap->getTsdfLayer().voxels_per_side());
    unsigned int counter = 0;

    // Get color.
    Color color = kUnknownColor_;
    const float alpha = 0.2f;
    auto vis_it = vis_infos_.find(submap->getID());
    if (vis_it != vis_infos_.end()) {
      color = vis_it->second.color;
    }

    for (auto& block_index : blocks) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = submap->getFrameName();
      marker.header.stamp = ros::Time::now();
      marker.color.r = color.r;
      marker.color.g = color.g;
      marker.color.b = color.b;
      marker.color.a = alpha;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.id = counter++;
      marker.ns = "tsdf_blocks_" + std::to_string(submap->getID());
      marker.scale.x = block_size;
      marker.scale.y = block_size;
      marker.scale.z = block_size;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      Point origin =
          submap->getTsdfLayer().getBlockByIndex(block_index).origin();
      marker.pose.position.x = origin.x() + block_size / 2.0;
      marker.pose.position.y = origin.y() + block_size / 2.0;
      marker.pose.position.z = origin.z() + block_size / 2.0;
      result.markers.push_back(marker);
    }
  }
  return result;
}

pcl::PointCloud<pcl::PointXYZI> SubmapVisualizer::generateFreeSpaceMsg(
    const SubmapCollection& submaps) {
  pcl::PointCloud<pcl::PointXYZI> result;

  // Create a pointcloud with distance = intensity. Taken from voxblox.
  int free_space_id = submaps.getActiveFreeSpaceSubmapID();
  if (submaps.submapIdExists(free_space_id)) {
    createDistancePointcloudFromTsdfLayer(
        submaps.getSubmap(free_space_id).getTsdfLayer(), &result);
  }
  return result;
}

visualization_msgs::MarkerArray SubmapVisualizer::generateBoundingVolumeMsgs(
    const SubmapCollection& submaps) {
  visualization_msgs::MarkerArray result;
  // Update the visualization infos.
  if (!vis_infos_are_updated_) {
    updateVisInfos(submaps);
  }

  for (const auto& submap : submaps) {
    if (submap->getLabel() == PanopticLabel::kFreeSpace &&
        !config_.include_free_space) {
      continue;
    }

    // Get color.
    Color color = kUnknownColor_;
    const float alpha = 0.2f;
    auto vis_it = vis_infos_.find(submap->getID());
    if (vis_it != vis_infos_.end()) {
      color = vis_it->second.color;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = submap->getFrameName();
    marker.header.stamp = ros::Time::now();
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = alpha;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.ns = "bounding_volume_" + std::to_string(submap->getID());
    marker.scale.x = submap->getBoundingVolume().getRadius() * 2.f;
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    const Point& origin = submap->getBoundingVolume().getCenter();
    marker.pose.position.x = origin.x();
    marker.pose.position.y = origin.y();
    marker.pose.position.z = origin.z();
    result.markers.push_back(marker);
  }
  return result;
}

void SubmapVisualizer::updateVisInfos(const SubmapCollection& submaps) {
  // Check whether the same submap collection is being visualized (cached data).
  if (previous_submaps_ != &submaps) {
    reset();
    previous_submaps_ = &submaps;
  }

  // Update submap ids.
  std::vector<int> ids;
  std::vector<int> new_ids;
  std::vector<int> deleted_ids;
  ids.reserve(vis_infos_.size());
  for (const auto& id_info_pair : vis_infos_) {
    ids.emplace_back(id_info_pair.first);
  }
  submaps.updateIDList(ids, &new_ids, &deleted_ids);

  // New submaps.
  for (int id : new_ids) {
    auto it = vis_infos_.emplace(std::make_pair(id, SubmapVisInfo())).first;
    SubmapVisInfo& info = it->second;
    info.id = id;
    info.republish_everything = true;
    setSubmapVisColor(submaps.getSubmap(id), &info);
  }

  // Deleted Submaps.
  for (int id : deleted_ids) {
    vis_infos_[id].was_deleted = true;
  }

  // Update colors where necessary.
  for (int id : ids) {
    setSubmapVisColor(submaps.getSubmap(id), &vis_infos_[id]);
  }
}

void SubmapVisualizer::setVisualizationMode(
    VisualizationMode visualization_mode) {
  // If there is a new visualization mode recompute the colors (alphas) and
  // republish everything.
  if (visualization_mode == visualization_mode_) {
    return;
  }
  visualization_mode_ = visualization_mode;
  reset();
}

void SubmapVisualizer::setColorMode(ColorMode color_mode) {
  // If there is a new color mode recompute the colors.
  // NOTE(schmluk): the modes 'color' and 'normals' are handled by the mesher,
  // so no need to recompute.
  if (color_mode == color_mode_) {
    return;
  }
  color_mode_ = color_mode;
  reset();
}

void SubmapVisualizer::setSubmapVisColor(const Submap& submap,
                                         SubmapVisInfo* info) {
  // Check whether colors need to be updated.
  if (info->was_deleted) {
    return;
  }

  // Update according to color mode.
  if (info->change_color || color_mode_ == ColorMode::kChange) {
    // NOTE(schmluk): Modes 'color' and 'normals' are handled by the mesher,
    // so no need to set here.
    switch (color_mode_) {
      case ColorMode::kInstances: {
        if (label_handler_->segmentationIdExists(submap.getInstanceID())) {
          info->color = label_handler_->getColor(submap.getInstanceID());
        } else {
          info->color = kUnknownColor_;
        }
        break;
      }
      case ColorMode::kSubmaps: {
        info->color = id_color_map_.colorLookup(info->id);
        break;
      }
      case ColorMode::kClasses: {
        if (submap.getLabel() == PanopticLabel::kUnknown) {
          info->color = kUnknownColor_;
        } else {
          info->color = id_color_map_.colorLookup(submap.getClassID());
        }
        break;
      }
      case ColorMode::kChange: {
        if (info->previous_change_state != submap.getChangeState() ||
            info->change_color) {
          switch (submap.getChangeState()) {
            case ChangeState::kNew: {
              info->color = Color(0, 200, 0);
              break;
            }
            case ChangeState::kMatched: {
              info->color = Color(0, 0, 255);
              break;
            }
            case ChangeState::kAbsent: {
              info->color = Color(255, 0, 0);
              break;
            }
            case ChangeState::kPersistent: {
              info->color = Color(0, 0, 255);
              break;
            }
            case ChangeState::kUnobserved: {
              info->color = Color(150, 150, 150);
              break;
            }
          }
          info->republish_everything = true;
          info->previous_change_state = submap.getChangeState();
        }
        break;
      }
    }
  }

  // Update according to visualization mode.
  if (info->change_color || visualization_mode_ == VisualizationMode::kActive) {
    switch (visualization_mode_) {
      case VisualizationMode::kAll: {
        info->alpha = 1.f;
        break;
      }
      case VisualizationMode::kActive: {
        if (info->was_active != submap.isActive() || info->change_color) {
          if (submap.isActive()) {
            info->alpha = 1.f;
          } else {
            info->alpha = 0.4f;
          }
          info->republish_everything = true;
          info->was_active = submap.isActive();
        }
        break;
      }
    }
  }

  info->change_color = false;
}

void SubmapVisualizer::publishTfTransforms(const SubmapCollection& submaps) {
  for (const auto& submap : submaps) {
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = global_frame_name_;
    msg.child_frame_id = submap->getFrameName();
    tf::transformKindrToMsg(submap->getT_S_M().cast<double>(), &msg.transform);
    tf_broadcaster_.sendTransform(msg);
  }
}

SubmapVisualizer::ColorMode SubmapVisualizer::colorModeFromString(
    const std::string& color_mode) {
  if (color_mode == "color") {
    return ColorMode::kColor;
  } else if (color_mode == "normals") {
    return ColorMode::kNormals;
  } else if (color_mode == "submaps") {
    return ColorMode::kSubmaps;
  } else if (color_mode == "instances") {
    return ColorMode::kInstances;
  } else if (color_mode == "classes") {
    return ColorMode::kClasses;
  } else if (color_mode == "change") {
    return ColorMode::kChange;
  } else {
    LOG(WARNING) << "Unknown ColorMode '" << color_mode
                 << "', using 'color' instead.";
    return ColorMode::kColor;
  }
}

std::string SubmapVisualizer::colorModeToString(ColorMode color_mode) {
  switch (color_mode) {
    case ColorMode::kColor:
      return "color";
    case ColorMode::kNormals:
      return "normals";
    case ColorMode::kSubmaps:
      return "submaps";
    case ColorMode::kInstances:
      return "instances";
    case ColorMode::kClasses:
      return "classes";
    case ColorMode::kChange:
      return "change";
  }
  return "unknown";
}

SubmapVisualizer::VisualizationMode
SubmapVisualizer::visualizationModeFromString(
    const std::string& visualization_mode) {
  if (visualization_mode == "all") {
    return VisualizationMode::kAll;
  } else if (visualization_mode == "active") {
    return VisualizationMode::kActive;
  } else {
    LOG(WARNING) << "Unknown VisualizationMode '" << visualization_mode
                 << "', using 'all' instead.";
    return VisualizationMode::kAll;
  }
}

std::string SubmapVisualizer::visualizationModeToString(
    VisualizationMode visualization_mode) {
  switch (visualization_mode) {
    case VisualizationMode::kAll:
      return "all";
    case VisualizationMode::kActive:
      return "active";
  }
  return "unknown";
}

}  // namespace panoptic_mapping
