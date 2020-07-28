#include "panoptic_mapping_ros/visualization/submap_visualizer.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <minkindr_conversions/kindr_msg.h>
#include <ros/time.h>
#include <voxblox_ros/mesh_vis.h>

namespace panoptic_mapping {

SubmapVisualizer::Config SubmapVisualizer::Config::isValid() const {
  CHECK_GT(submap_color_discretization, 0)
      << "The submap color discretization is expected > 0.";
  // NOTE(schmluk): if the coloring mode is not valid it will be defaulted to
  // 'color' and a warning will be raised.
  return Config(*this);
}

SubmapVisualizer::SubmapVisualizer(const Config& config,
                                   std::shared_ptr<LabelHandler> label_handler)
    : config_(config.isValid()),
      label_handler_(std::move(label_handler)),
      global_frame_name_("mission") {
  coloring_mode_ =
      SubmapVisualizer::coloringModeFromString(config_.mesh_coloring_mode);
}

void SubmapVisualizer::reset() { vis_infos_.clear(); }

void SubmapVisualizer::generateMeshMsgs(
    SubmapCollection* submaps, std::vector<voxblox_msgs::MultiMesh>* output) {
  if (!config_.visualize_mesh) {
    return;
  }
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(output);

  std::cout << "Submaps: ";
  for (auto& s : *submaps) {
    std::cout << s->getID() << "-" << s->getInstanceID() << ", ";
  }
  std::cout << std::endl;

  // update the visualization infos
  updateVisInfos(*submaps);

  // process all submaps based on their visualization info
  for (const auto& submap : *submaps) {
    // find the corresponding info.
    auto it = vis_infos_.find(submap->getID());
    if (it == vis_infos_.end()) {
      LOG(WARNING) << "Tried to visualize submap " << submap->getID()
                   << " without existing SubmapVisInfo.";
      continue;
    }
    SubmapVisInfo& info = it->second;

    // recompute colors if requested
    if (info.change_color) {
      setSubmapVisColor(*submap, &info);
    }

    // setup message
    voxblox_msgs::MultiMesh msg;
    msg.header.stamp = ros::Time::now();
    msg.id = info.id;

    // If the submap was deleted we send an empty message to delete the visual.
    if (info.was_deleted) {
      output->push_back(msg);
      vis_infos_.erase(it);
      continue;
    }

    // update the mesh
    msg.header.frame_id = submap->getFrameName();
    updateSubmapMesh(submap.get(), info.remesh_everything);

    // mark the whole mesh for re-publishing if requested
    if (info.republish_everything) {
      voxblox::BlockIndexList mesh_indices;
      submap->getMeshLayer().getAllAllocatedMeshes(&mesh_indices);
      for (const auto& block_index : mesh_indices) {
        submap->getMeshLayerPtr()->getMeshPtrByIndex(block_index)->updated =
            true;
      }
    }

    // Set the voxblox internal color mode. Gray will be used for overwriting.
    voxblox::ColorMode color_mode_voxblox = voxblox::ColorMode::kGray;
    if (coloring_mode_ == MeshColoringMode::kColor) {
      color_mode_voxblox = voxblox::ColorMode::kColor;
    } else if (coloring_mode_ == MeshColoringMode::kNormals) {
      color_mode_voxblox = voxblox::ColorMode::kNormals;
    }
    voxblox::generateVoxbloxMeshMsg(submap->getMeshLayerPtr(),
                                    color_mode_voxblox, &msg.mesh);

    // Apply the submap color if necessary
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

    // set alpha values
    msg.alpha = 255;
    output->emplace_back(std::move(msg));

    // publish submap transforms
    publishTfTransform(submap->getT_M_S(), global_frame_name_,
                       submap->getFrameName());

    // wrap up
    info.republish_everything = false;
    info.remesh_everything = false;
    info.change_color = false;
  }
}

void SubmapVisualizer::updateVisInfos(const SubmapCollection& submaps) {
  for (const auto& submap : submaps) {
    auto it = vis_infos_.find(submap->getID());
    if (it == vis_infos_.end()) {
      // New submap
      it = vis_infos_.emplace(std::make_pair(submap->getID(), SubmapVisInfo()))
               .first;
      SubmapVisInfo& info = it->second;
      info.id = submap->getID();
      info.change_color = true;
      info.remesh_everything = true;  // could be a loaded submap
    }
  }
  for (auto& info_pair : vis_infos_) {
    if (!submaps.submapIdExists(info_pair.second.id)) {
      // Deleted submap
      info_pair.second.was_deleted = true;
    }
  }
}

void SubmapVisualizer::generateBlockMsgs(
    const SubmapCollection& submaps,
    visualization_msgs::MarkerArray* output) const {
  if (!config_.visualize_tsdf_blocks) {
    return;
  }

  CHECK_NOTNULL(output);
  for (const auto& submap : submaps) {
    // setup submap
    voxblox::BlockIndexList blocks;
    submap->getTsdfLayer().getAllAllocatedBlocks(&blocks);
    float block_size =
        submap->getTsdfLayer().voxel_size() *
        static_cast<float>(submap->getTsdfLayer().voxels_per_side());
    int counter = 0;

    // get color
    Color color = kUnknownColor;
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
      marker.color.a = 0.5;
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
      output->markers.push_back(marker);
    }
  }
}

void SubmapVisualizer::setMeshColoringMode(MeshColoringMode coloring_mode) {
  // if there is a new coloring mode recompute the colors.
  // NOTE(schmluk): the modes 'color' and 'normals' are handled by the mesher,
  // so no need to recompute.
  if (coloring_mode != coloring_mode_ &&
      coloring_mode != MeshColoringMode::kColor &&
      coloring_mode != MeshColoringMode::kNormals) {
    for (auto& info : vis_infos_) {
      info.second.change_color = true;
    }
  }

  coloring_mode_ = coloring_mode;
}

void SubmapVisualizer::setSubmapVisColor(const Submap& submap,
                                         SubmapVisInfo* info) {
  if (submap.getID() != info->id) {
    LOG(WARNING) << "Set color of SubmapVisInfo " << info->id
                 << " based on Submap " << submap.getID() << ".";
  }

  // NOTE(schmluk): modes 'color' and 'normals' are handled by the mesher,
  // so no need to set here.
  switch (coloring_mode_) {
    case MeshColoringMode::kInstances: {
      if (label_handler_->segmentationIdExists(submap.getInstanceID())) {
        info->color = label_handler_->getColor(submap.getInstanceID());
      } else {
        info->color = kUnknownColor;
      }
      break;
    }
    case MeshColoringMode::kSubmaps: {
      float h = static_cast<float>(submap.getID() %
                                   config_.submap_color_discretization) /
                static_cast<float>(config_.submap_color_discretization - 1);
      info->color = voxblox::rainbowColorMap(h);
      break;
    }
    case MeshColoringMode::kClasses: {
      LOG(WARNING) << "Class coloring is not yet implemented.";
      break;
    }
  }
}

void SubmapVisualizer::updateSubmapMesh(Submap* submap,
                                        bool update_all_blocks) {
  CHECK_NOTNULL(submap);
  constexpr bool clear_updated_flag = true;
  mesh_integrator_ = std::make_unique<voxblox::MeshIntegrator<TsdfVoxel>>(
      config_.mesh_integrator_config, submap->getTsdfLayerPtr().get(),
      submap->getMeshLayerPtr().get());
  mesh_integrator_->generateMesh(!update_all_blocks, clear_updated_flag);
}

void SubmapVisualizer::publishTfTransform(const Transformation& transform,
                                          const std::string& parent_frame,
                                          const std::string& child_frame) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = parent_frame;
  msg.child_frame_id = child_frame;
  tf::transformKindrToMsg(transform.cast<double>(), &msg.transform);
  tf_broadcaster_.sendTransform(msg);
}

SubmapVisualizer::MeshColoringMode SubmapVisualizer::coloringModeFromString(
    const std::string& coloring_mode) {
  if (coloring_mode == "color") {
    return MeshColoringMode::kColor;
  } else if (coloring_mode == "normals") {
    return MeshColoringMode::kNormals;
  } else if (coloring_mode == "submaps") {
    return MeshColoringMode::kSubmaps;
  } else if (coloring_mode == "instances") {
    return MeshColoringMode::kInstances;
  } else if (coloring_mode == "classes") {
    return MeshColoringMode::kClasses;
  } else {
    LOG(WARNING) << "Unknown MeshColoringMode '" << coloring_mode
                 << "', using color instead.";
    return MeshColoringMode::kColor;
  }
}

std::string SubmapVisualizer::coloringModeToString(
    MeshColoringMode coloring_mode) {
  switch (coloring_mode) {
    case MeshColoringMode::kColor:
      return "color";
    case MeshColoringMode::kNormals:
      return "normals";
    case MeshColoringMode::kSubmaps:
      return "submaps";
    case MeshColoringMode::kInstances:
      return "instances";
    case MeshColoringMode::kClasses:
      return "classes";
  }
  return "unknown coloring mode";
}

}  // namespace panoptic_mapping
