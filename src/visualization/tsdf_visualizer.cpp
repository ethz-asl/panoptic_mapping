#include "panoptic_mapping/visualization/tsdf_visualizer.h"

#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ros_params.h>
#include <ros/time.h>

namespace panoptic_mapping {

bool TsdfVisualizer::setupFromRos(const ros::NodeHandle &nh) {
  config_ = voxblox::getMeshIntegratorConfigFromRosParam(nh);
}

bool TsdfVisualizer::updatehMesh(Submap *submap, bool update_all_blocks) {
  CHECK_NOTNULL(submap);
  constexpr bool clear_updated_flag = true;
  mesh_integrator_.reset(new voxblox::MeshIntegrator<TsdfVoxel>(config_,
                                                                submap->getTsdfLayerPtr(),
                                                                submap->getMeshLayerPtr()));
  mesh_integrator_->generateMesh(!update_all_blocks, clear_updated_flag);
  return true;
}

void TsdfVisualizer::generateMeshMsg(Submap *submap, voxblox_msgs::MultiMesh *output, bool mesh_all_blocks) {
  CHECK_NOTNULL(output);
  // set updated flag if necessary
  if (mesh_all_blocks){
    voxblox::BlockIndexList mesh_indices;
    submap->getMeshLayer().getAllAllocatedMeshes(&mesh_indices);
    for (const auto& block_index : mesh_indices) {
      submap->getMeshLayerPtr()->getMeshPtrByIndex(block_index)->updated = true;
    }
  }

  // Coloring mode inside voxblox
  voxblox::ColorMode color_mode = voxblox::ColorMode::kColor;
  if (mesh_coloring_method_ == "normals") {
    color_mode = voxblox::ColorMode::kNormals;
  }

  // generate mesh
  generateVoxbloxMeshMsg(submap->getMeshLayerPtr(), color_mode, &(output->mesh));
  output->header.frame_id = submap->getFrameName();
  output->header.stamp = ros::Time::now();
  output->id = submap->getID();
  output->alpha = 255;

  // otherwise recolor according to submap
  if (mesh_coloring_method_ == "submap_color") {
    for (auto &mesh_block : output->mesh.mesh_blocks) {
      for (auto &r : mesh_block.r) {
        r = submap->getColor().r;
      }
      for (auto &g : mesh_block.g) {
        g = submap->getColor().g;
      }
      for (auto &b : mesh_block.b) {
        b = submap->getColor().b;
      }
    }
  }
}

void TsdfVisualizer::generateBlockMsg(const Submap &submap, visualization_msgs::MarkerArray *output) {
  CHECK_NOTNULL(output);
  voxblox::BlockIndexList blocks;
  submap.getTsdfLayer().getAllAllocatedBlocks(&blocks);
  int counter = 0;
  for (auto &block_index : blocks) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = submap.getFrameName();
    marker.header.stamp = ros::Time::now();
    marker.color.r = submap.getColor().r;
    marker.color.g = submap.getColor().g;
    marker.color.b = submap.getColor().b;
    marker.color.a = 0.5;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.id = counter++;
    marker.ns = "tsdf_blocks_" + std::to_string(submap.getID());
    float block_size = submap.getTsdfLayer().voxel_size() * (float)submap.getTsdfLayer().voxels_per_side();
    marker.scale.x = block_size;
    marker.scale.y = block_size;
    marker.scale.z = block_size;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    Point origin = submap.getTsdfLayer().getBlockByIndex(block_index).origin();
    marker.pose.position.x = origin.x() + block_size / 2.0;
    marker.pose.position.y = origin.y() + block_size / 2.0;
    marker.pose.position.z = origin.z() + block_size / 2.0;
    output->markers.push_back(marker);
  }
}

} // namespace panoptic_mapping
