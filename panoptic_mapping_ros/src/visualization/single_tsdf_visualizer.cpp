#include "panoptic_mapping_ros/visualization/single_tsdf_visualizer.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "panoptic_mapping/map/classification/fixed_count.h"
#include "panoptic_mapping/map/classification/uncertainty.h"
#include "panoptic_mapping/tools/coloring.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<
    SubmapVisualizer, SingleTsdfVisualizer, std::shared_ptr<Globals>>
    SingleTsdfVisualizer::registration_("single_tsdf");

void SingleTsdfVisualizer::Config::checkParams() const {
  checkParamConfig(submap_visualizer);
}

void SingleTsdfVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("submap_visualizer", &submap_visualizer);
  setupParam("entropy_factor", &entropy_factor);
}

SingleTsdfVisualizer::SingleTsdfVisualizer(const Config& config,
                                           std::shared_ptr<Globals> globals,
                                           bool print_config)
    : SubmapVisualizer(config.submap_visualizer, std::move(globals), false),
      config_(config.checkValid()) {
  // Print config after setting up the modes.
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
  // Check the visualization modes.
  setVisualizationMode(visualization_mode_);
  setColorMode(color_mode_);
}

void SingleTsdfVisualizer::reset() {
  // Erase all current tracking / cached data.
  info_ = SubmapVisInfo();
  info_.republish_everything = true;
  previous_submaps_ = nullptr;
}

void SingleTsdfVisualizer::clearMesh() {
  // Clear the current mesh from the rviz plugin.
  if (config_.submap_visualizer.visualize_mesh &&
      mesh_pub_.getNumSubscribers() > 0) {
    voxblox_msgs::MultiMesh msg;
    msg.header.stamp = ros::Time::now();
    msg.name_space = map_name_space_;
    mesh_pub_.publish(msg);
  }
}

std::vector<voxblox_msgs::MultiMesh> SingleTsdfVisualizer::generateMeshMsgs(
    SubmapCollection* submaps) {
  std::vector<voxblox_msgs::MultiMesh> result;
  if (submaps->size() == 0) {
    LOG(WARNING) << "No Map to visualize.";
    return result;
  }
  // If the freespace map ID is not yet setup (loaded map) assume it's the first
  // submap in the collection.
  if (submaps->getActiveFreeSpaceSubmapID() < 0) {
    submaps->setActiveFreeSpaceSubmapID(submaps->begin()->getID());
  }

  // Get the single map.
  Submap& submap =
      *submaps->getSubmapPtr(submaps->getActiveFreeSpaceSubmapID());

  // Setup message.
  voxblox_msgs::MultiMesh msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = submap.getFrameName();
  msg.name_space = map_name_space_;

  // Update the mesh.
  submap.updateMesh(true, false);

  // Mark the whole mesh for re-publishing if requested.
  if (info_.republish_everything) {
    voxblox::BlockIndexList mesh_indices;
    submap.getMeshLayer().getAllAllocatedMeshes(&mesh_indices);
    for (const auto& block_index : mesh_indices) {
      submap.getMeshLayerPtr()->getMeshPtrByIndex(block_index)->updated = true;
    }
    info_.republish_everything = false;
  }

  // Set the voxblox internal color mode. Gray will be used for overwriting.
  voxblox::ColorMode color_mode_voxblox = voxblox::ColorMode::kGray;
  if (color_mode_ == ColorMode::kColor) {
    color_mode_voxblox = voxblox::ColorMode::kColor;
  } else if (color_mode_ == ColorMode::kNormals) {
    color_mode_voxblox = voxblox::ColorMode::kNormals;
  }

  voxblox::generateVoxbloxMeshMsg(submap.getMeshLayerPtr(), color_mode_voxblox,
                                  &msg.mesh);

  // Add removed blocks so they are cleared from the visualization as well.
  voxblox::BlockIndexList block_indices;
  submap.getTsdfLayer().getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : info_.previous_blocks) {
    if (std::find(block_indices.begin(), block_indices.end(), block_index) ==
        block_indices.end()) {
      voxblox_msgs::MeshBlock mesh_block;
      mesh_block.index[0] = block_index.x();
      mesh_block.index[1] = block_index.y();
      mesh_block.index[2] = block_index.z();
      msg.mesh.mesh_blocks.push_back(mesh_block);
    }
  }
  info_.previous_blocks = block_indices;

  if (msg.mesh.mesh_blocks.empty()) {
    // Nothing changed, don't send an empty msg which would reset the mesh.
    return result;
  }

  // Apply the submap color if necessary.
  if (color_mode_voxblox == voxblox::ColorMode::kGray) {
    if (!submap.hasClassLayer()) {
      LOG_IF(WARNING, config_.verbosity >= 2)
          << "Can not create color for mode '" << colorModeToString(color_mode_)
          << "' without existing class layer.";
    } else {
      for (auto& mesh_block : msg.mesh.mesh_blocks) {
        colorMeshBlock(submap, &mesh_block);
      }
    }
  }

  result.emplace_back(std::move(msg));
  return result;
}

void SingleTsdfVisualizer::colorMeshBlock(const Submap& submap,
                                          voxblox_msgs::MeshBlock* mesh_block) {
  const voxblox::BlockIndex block_index(
      mesh_block->index[0], mesh_block->index[1], mesh_block->index[2]);
  if (!submap.getClassLayer().hasBlock(block_index)) {
    return;
  }

  // Setup.
  const ClassBlock::ConstPtr class_block =
      submap.getClassLayer().getBlockConstPtrByIndex(block_index);
  const float point_conv_factor = 2.f / std::numeric_limits<uint16_t>::max();
  const float block_edge_length = submap.getClassLayer().block_size();
  const size_t num_vertices = mesh_block->x.size();
  mesh_block->r.resize(num_vertices);
  mesh_block->g.resize(num_vertices);
  mesh_block->b.resize(num_vertices);

  // Coloring schemes.
  std::function<Color(const ClassVoxel&)> get_color = getColoring();

  for (int i = 0; i < num_vertices; ++i) {
    // Get the vertex position in map frame.
    const float mesh_x =
        (static_cast<float>(mesh_block->x[i]) * point_conv_factor +
         static_cast<float>(block_index[0])) *
        block_edge_length;
    const float mesh_y =
        (static_cast<float>(mesh_block->y[i]) * point_conv_factor +
         static_cast<float>(block_index[1])) *
        block_edge_length;
    const float mesh_z =
        (static_cast<float>(mesh_block->z[i]) * point_conv_factor +
         static_cast<float>(block_index[2])) *
        block_edge_length;
    const ClassVoxel& voxel =
        class_block->getVoxelByCoordinates({mesh_x, mesh_y, mesh_z});
    const Color color = get_color(voxel);
    mesh_block->r[i] = color.r;
    mesh_block->g[i] = color.g;
    mesh_block->b[i] = color.b;
  }
}

std::function<Color(const ClassVoxel&)> SingleTsdfVisualizer::getColoring()
    const {
  switch (color_mode_) {
    case ColorMode::kClassification:
      return [](const ClassVoxel& voxel) {
        // Look up the probability of the voxel belonging to its highest rating
        // entity.
        return redToGreenGradient(voxel.getProbability(voxel.getBelongingID()));
      };

    case ColorMode::kUncertainty:
      return [this](const ClassVoxel& voxel) {
        if (voxel.getVoxelType() != ClassVoxelType::kUncertainty) {
          return kUnknownColor_;
        }
        const UncertaintyVoxel& uncertainty_voxel =
            static_cast<const UncertaintyVoxel&>(voxel);
        if (uncertainty_voxel.is_ground_truth) {
          return Color(0, 0, 255);
        }
        float probability = uncertainty_voxel.uncertainty;
        // Well defined uncertanties should never be > 1.
        if (probability > 1.f) {
          probability = 1.f;
        }
        return redToGreenGradient(probability);
      };

    case ColorMode::kEntropy:
      return [this](const ClassVoxel& voxel) {
        if (voxel.getVoxelType() != ClassVoxelType::kUncertainty &&
            voxel.getVoxelType() != ClassVoxelType::kFixedCount) {
          // NOTE(schmluk): This is easy to implement also for other types of
          // voxels but not yet supported.
          return kUnknownColor_;
        }
        const FixedCountVoxel& count_voxel =
            static_cast<const FixedCountVoxel&>(voxel);
        if (count_voxel.counts.empty()) {
          return Color(255, 0, 0);
        }
        const float uniform_prob =
            1.f / static_cast<float>(count_voxel.counts.size());
        const float uniform_entropy = -count_voxel.counts.size() *
                                      (uniform_prob * std::log(uniform_prob));
        float normalized_entropy =
            -std::accumulate(
                count_voxel.counts.begin(), count_voxel.counts.end(), 0.f,
                [count_voxel](float accumulated,
                              ClassificationCount new_value) {
                  const float to_add =
                      static_cast<float>(new_value) / count_voxel.total_count;
                  if (to_add <= 0.f) {
                    return accumulated;
                  }
                  return accumulated + to_add * std::log(to_add);
                }) /
            std::log(count_voxel.counts.size());
        normalized_entropy =
            normalized_entropy / uniform_entropy *
            config_.entropy_factor;  // Entropies are often very small. Use
                                     // this to make small values visible
        if (normalized_entropy > 1.f) {
          normalized_entropy = 1;
        }

        return redToGreenGradient(normalized_entropy);
      };
    default:
      return [this](const ClassVoxel& voxel) {
        // This implies the visualization mode is instances or classes.
        return id_color_map_.colorLookup(voxel.getBelongingID());
      };
  }
}

void SingleTsdfVisualizer::updateVisInfos(const SubmapCollection& submaps) {
  // Check whether the same submap collection is being visualized (cached
  // data).
  if (previous_submaps_ != &submaps) {
    reset();
    previous_submaps_ = &submaps;
  }
}

void SingleTsdfVisualizer::setVisualizationMode(
    VisualizationMode visualization_mode) {
  // If there is a new visualization mode recompute the colors and
  // republish everything.
  if (visualization_mode != VisualizationMode::kAll) {
    LOG(WARNING) << "Visualization mode '"
                 << visualizationModeToString(visualization_mode)
                 << "' is not supported, using 'all' instead.";
    visualization_mode_ = VisualizationMode::kAll;
  } else {
    visualization_mode_ = visualization_mode;
  }
  reset();
}

void SingleTsdfVisualizer::setColorMode(ColorMode color_mode) {
  // If there is a new color mode recompute the colors.
  if (color_mode == ColorMode::kChange || color_mode == ColorMode::kSubmaps) {
    LOG(WARNING) << "Color mode '" << colorModeToString(color_mode)
                 << "' is not supported, using 'color' instead.";
    color_mode_ = ColorMode::kColor;
  } else {
    color_mode_ = color_mode;
  }
  reset();
}

}  // namespace panoptic_mapping
