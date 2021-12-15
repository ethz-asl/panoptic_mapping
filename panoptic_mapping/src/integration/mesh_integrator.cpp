#include "panoptic_mapping/integration/mesh_integrator.h"

#include <chrono>
#include <future>
#include <list>
#include <memory>
#include <utility>
#include <vector>

#include <voxblox/interpolator/interpolator.h>
#include <voxblox/mesh/marching_cubes.h>
#include <voxblox/utils/meshing_utils.h>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

void MeshIntegrator::Config::checkParams() const {
  checkParamGT(min_weight, 0.f, "min_weight");
  checkParamGE(required_belonging_corners, 0, "required_belonging_corners");
  checkParamLE(required_belonging_corners, 8, "required_belonging_corners");
  checkParamGT(integrator_threads, 0, "integrator_threads");
}

void MeshIntegrator::Config::setupParamsAndPrinting() {
  setupParam("use_color", &use_color);
  setupParam("min_weight", &min_weight);
  setupParam("required_belonging_corners", &required_belonging_corners);
  setupParam("clear_foreign_voxels", &clear_foreign_voxels);
  setupParam("integrator_threads", &integrator_threads);
}

MeshIntegrator::MeshIntegrator(const MeshIntegrator::Config& config,
                               std::shared_ptr<TsdfLayer> tsdf_layer,
                               std::shared_ptr<MeshLayer> mesh_layer,
                               std::shared_ptr<ClassLayer> class_layer,
                               float truncation_distance)
    : config_(config.checkValid()),
      tsdf_layer_(std::move(tsdf_layer)),
      mesh_layer_(std::move(mesh_layer)),
      class_layer_(std::move(class_layer)),
      truncation_distance_(truncation_distance) {
  // Check input is valid (class layer can be null).
  if (!tsdf_layer_) {
    LOG(ERROR) << "The TSDF layer may not be uninitialized!";
    return;
  } else if (!mesh_layer_) {
    LOG(ERROR) << "The mesh layer may not be uninitialized!";
    return;
  } else if (tsdf_layer_->voxel_size() * tsdf_layer_->voxels_per_side() -
                 mesh_layer_->block_size() >
             1e-6) {
    LOG(ERROR) << "TSDF and Mesh layers have different layouts!";
    return;
  }
  if (class_layer_) {
    if (truncation_distance_ <= 0.f) {
      LOG(ERROR) << "The truncation distance must be > 0.0!";
      return;
    }
    if (class_layer_->voxel_size() != tsdf_layer_->voxel_size() ||
        class_layer_->voxels_per_side() != tsdf_layer_->voxels_per_side()) {
      LOG(ERROR) << "TSDF and Class layers have different layouts!";
      return;
    }
  }

  // Cache input data.
  voxel_size_ = tsdf_layer_->voxel_size();
  voxels_per_side_ = tsdf_layer_->voxels_per_side();
  cube_index_offsets_ << 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
      0, 0, 1, 1, 1, 1;
}

void MeshIntegrator::generateMesh(bool only_mesh_updated_blocks,
                                  bool clear_updated_flag,
                                  bool use_class_data) {
  use_class_layer_ = use_class_data;
  if (!class_layer_ && use_class_layer_) {
    use_class_layer_ = false;
    LOG(WARNING) << "Tried to use un-initialized class layer, will be ignored.";
  }
  voxblox::BlockIndexList tsdf_blocks;
  if (only_mesh_updated_blocks) {
    tsdf_layer_->getAllUpdatedBlocks(voxblox::Update::Status::kMesh,
                                     &tsdf_blocks);
  } else {
    tsdf_layer_->getAllAllocatedBlocks(&tsdf_blocks);
  }

  // Allocate all the mesh memory.
  for (const voxblox::BlockIndex& block_index : tsdf_blocks) {
    mesh_layer_->allocateMeshPtrByIndex(block_index);
  }

  std::unique_ptr<voxblox::ThreadSafeIndex> index_getter(
      new voxblox::MixedThreadSafeIndex(tsdf_blocks.size()));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(
        &MeshIntegrator::generateMeshBlocksFunction, this, tsdf_blocks,
        clear_updated_flag, index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }
}

void MeshIntegrator::generateMeshBlocksFunction(
    const voxblox::BlockIndexList& tsdf_blocks, bool clear_updated_flag,
    voxblox::ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    const voxblox::BlockIndex& block_idx = tsdf_blocks[list_idx];
    const bool success = updateMeshForBlock(block_idx);
    if (clear_updated_flag && success) {
      tsdf_layer_->getBlockPtrByIndex(block_idx)->setUpdated(
          voxblox::Update::Status::kMesh, false);
    }
  }
}

bool MeshIntegrator::updateMeshForBlock(
    const voxblox::BlockIndex& block_index) {
  voxblox::Mesh::Ptr mesh = mesh_layer_->getMeshPtrByIndex(block_index);
  mesh->clear();
  // This block should already exist, otherwise it makes no sense to update
  // the mesh for it. ;)
  if (!tsdf_layer_->hasBlock(block_index)) {
    LOG(WARNING) << "Trying to mesh a non-existent TSDF block at index: "
                 << block_index.transpose() << ", skipping block.";
    return false;
  }
  const TsdfBlock& tsdf_block = tsdf_layer_->getBlockByIndex(block_index);
  // The class is accessed by pointer since it's just a nullptr if the class
  // info is not used.
  ClassBlock::ConstPtr class_block;
  if (use_class_layer_) {
    class_block = class_layer_->getBlockConstPtrByIndex(block_index);
    if (!class_block) {
      LOG(WARNING) << "Trying to mesh a non-existent class block at index: "
                   << block_index.transpose() << ", skipping block.";
      return false;
    }
  }

  extractBlockMesh(tsdf_block, class_block, mesh.get());

  // Update colors if needed.
  if (config_.use_color) {
    updateMeshColor(tsdf_block, class_block, mesh.get());
  }

  mesh->updated = true;
  return true;
}

void MeshIntegrator::extractBlockMesh(const TsdfBlock& tsdf_block,
                                      const ClassBlock::ConstPtr& class_block,
                                      voxblox::Mesh* mesh) {
  DCHECK(mesh != nullptr);

  voxblox::IndexElement vps = tsdf_block.voxels_per_side();
  voxblox::VertexIndex next_mesh_index = 0;

  voxblox::VoxelIndex voxel_index;
  for (voxel_index.x() = 0; voxel_index.x() < vps - 1; ++voxel_index.x()) {
    for (voxel_index.y() = 0; voxel_index.y() < vps - 1; ++voxel_index.y()) {
      for (voxel_index.z() = 0; voxel_index.z() < vps - 1; ++voxel_index.z()) {
        Point coords = tsdf_block.computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshInsideBlock(tsdf_block, class_block, voxel_index, coords,
                               &next_mesh_index, mesh);
      }
    }
  }

  // Max X plane
  // takes care of edge (x_max, y_max, z),
  // takes care of edge (x_max, y, z_max).
  voxel_index.x() = vps - 1;
  for (voxel_index.z() = 0; voxel_index.z() < vps; voxel_index.z()++) {
    for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
      Point coords = tsdf_block.computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(tsdf_block, class_block, voxel_index, coords,
                          &next_mesh_index, mesh);
    }
  }

  // Max Y plane.
  // takes care of edge (x, y_max, z_max),
  // without corner (x_max, y_max, z_max).
  voxel_index.y() = vps - 1;
  for (voxel_index.z() = 0; voxel_index.z() < vps; voxel_index.z()++) {
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
      Point coords = tsdf_block.computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(tsdf_block, class_block, voxel_index, coords,
                          &next_mesh_index, mesh);
    }
  }

  // Max Z plane.
  voxel_index.z() = vps - 1;
  for (voxel_index.y() = 0; voxel_index.y() < vps - 1; voxel_index.y()++) {
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
      Point coords = tsdf_block.computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(tsdf_block, class_block, voxel_index, coords,
                          &next_mesh_index, mesh);
    }
  }
}

void MeshIntegrator::extractMeshInsideBlock(
    const TsdfBlock& tsdf_block, const ClassBlock::ConstPtr& class_block,
    const voxblox::VoxelIndex& index, const Point& coords,
    voxblox::VertexIndex* next_mesh_index, voxblox::Mesh* mesh) {
  Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets =
      cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
  Eigen::Matrix<FloatingPoint, 3, 8> corner_coords;
  Eigen::Matrix<FloatingPoint, 8, 1> corner_sdf;
  Eigen::Matrix<bool, 8, 1> corner_belongs;
  bool all_neighbors_observed = true;
  int belonging_corners = 0;
  const bool use_class = class_block;

  for (unsigned int i = 0; i < 8; ++i) {
    // Get all sdf values.
    voxblox::VoxelIndex corner_index = index + cube_index_offsets_.col(i);
    const TsdfVoxel& voxel = tsdf_block.getVoxelByVoxelIndex(corner_index);

    if (!voxblox::utils::getSdfIfValid(voxel, config_.min_weight,
                                       &(corner_sdf(i)))) {
      all_neighbors_observed = false;
      break;
    }
    if (use_class) {
      corner_belongs(i) =
          class_block->getVoxelByVoxelIndex(corner_index).belongsToSubmap();
      if (corner_belongs(i)) {
        belonging_corners++;
      }
    }

    corner_coords.col(i) = coords + cube_coord_offsets.col(i);
  }

  if (all_neighbors_observed) {
    if (use_class && belonging_corners <= config_.required_belonging_corners) {
      return;
    }

    if (use_class && config_.clear_foreign_voxels) {
      // Foreign voxels are set to truncation distance to close the mesh.
      for (unsigned int i = 0; i < 8; ++i) {
        if (!corner_belongs(i)) {
          corner_sdf(i) = truncation_distance_;
        }
      }
    }
    voxblox::MarchingCubes::meshCube(corner_coords, corner_sdf, next_mesh_index,
                                     mesh);
  }
}

void MeshIntegrator::extractMeshOnBorder(
    const TsdfBlock& tsdf_block, const ClassBlock::ConstPtr& class_block,
    const voxblox::VoxelIndex& index, const Point& coords,
    voxblox::VertexIndex* next_mesh_index, voxblox::Mesh* mesh) {
  Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets =
      cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
  Eigen::Matrix<FloatingPoint, 3, 8> corner_coords;
  Eigen::Matrix<FloatingPoint, 8, 1> corner_sdf;
  Eigen::Matrix<bool, 8, 1> corner_belongs;
  bool all_neighbors_observed = true;
  int belonging_corners = 0;
  corner_coords.setZero();
  corner_sdf.setZero();
  const bool use_class = class_block;

  for (unsigned int i = 0; i < 8; ++i) {
    voxblox::VoxelIndex corner_index = index + cube_index_offsets_.col(i);

    if (tsdf_block.isValidVoxelIndex(corner_index)) {
      const TsdfVoxel& voxel = tsdf_block.getVoxelByVoxelIndex(corner_index);

      if (!voxblox::utils::getSdfIfValid(voxel, config_.min_weight,
                                         &(corner_sdf(i)))) {
        all_neighbors_observed = false;
        break;
      }
      if (use_class) {
        corner_belongs(i) =
            class_block->getVoxelByVoxelIndex(corner_index).belongsToSubmap();
        if (corner_belongs(i)) {
          belonging_corners++;
        }
      }

      corner_coords.col(i) = coords + cube_coord_offsets.col(i);
    } else {
      // We have to access a different block.
      voxblox::BlockIndex block_offset = voxblox::BlockIndex::Zero();

      for (unsigned int j = 0u; j < 3u; j++) {
        if (corner_index(j) < 0) {
          block_offset(j) = -1;
          corner_index(j) = corner_index(j) + voxels_per_side_;
        } else if (corner_index(j) >=
                   static_cast<voxblox::IndexElement>(voxels_per_side_)) {
          block_offset(j) = 1;
          corner_index(j) = corner_index(j) - voxels_per_side_;
        }
      }

      voxblox::BlockIndex neighbor_index =
          tsdf_block.block_index() + block_offset;

      if (tsdf_layer_->hasBlock(neighbor_index)) {
        const TsdfBlock& neighbor_block =
            tsdf_layer_->getBlockByIndex(neighbor_index);

        CHECK(neighbor_block.isValidVoxelIndex(corner_index));
        const TsdfVoxel& voxel =
            neighbor_block.getVoxelByVoxelIndex(corner_index);

        if (!voxblox::utils::getSdfIfValid(voxel, config_.min_weight,
                                           &(corner_sdf(i)))) {
          all_neighbors_observed = false;
          break;
        }
        if (use_class_layer_) {
          // The class blocks should always exist but just make sure.
          const ClassBlock::ConstPtr neighbor_class_block =
              class_layer_->getBlockConstPtrByIndex(neighbor_index);
          corner_belongs(i) =
              neighbor_class_block
                  ? neighbor_class_block->getVoxelByVoxelIndex(corner_index)
                        .belongsToSubmap()
                  : true;
          if (corner_belongs(i)) {
            belonging_corners++;
          }
        }
        corner_coords.col(i) = coords + cube_coord_offsets.col(i);
      } else {
        all_neighbors_observed = false;
        break;
      }
    }
  }

  if (all_neighbors_observed) {
    if (use_class_layer_ &&
        belonging_corners <= config_.required_belonging_corners) {
      return;
    }

    if (use_class_layer_ && config_.clear_foreign_voxels) {
      // Foreign voxels are set to truncation distance to close the mesh.
      for (unsigned int i = 0; i < 8; ++i) {
        if (!corner_belongs(i)) {
          corner_sdf(i) = truncation_distance_;
        }
      }
    }
    voxblox::MarchingCubes::meshCube(corner_coords, corner_sdf, next_mesh_index,
                                     mesh);
  }
}

void MeshIntegrator::updateMeshColor(const TsdfBlock& tsdf_block,
                                     const ClassBlock::ConstPtr& class_block,
                                     voxblox::Mesh* mesh) {
  mesh->colors.clear();
  mesh->colors.resize(mesh->indices.size());

  // Use nearest-neighbor search. Currently just use the tsdf stored colors.
  for (size_t i = 0; i < mesh->vertices.size(); i++) {
    const Point& vertex = mesh->vertices[i];
    voxblox::VoxelIndex voxel_index =
        tsdf_block.computeVoxelIndexFromCoordinates(vertex);
    if (tsdf_block.isValidVoxelIndex(voxel_index)) {
      const TsdfVoxel& voxel = tsdf_block.getVoxelByVoxelIndex(voxel_index);
      voxblox::utils::getColorIfValid(voxel, config_.min_weight,
                                      &(mesh->colors[i]));
    } else {
      const voxblox::BlockIndex index =
          tsdf_layer_->computeBlockIndexFromCoordinates(vertex);
      if (!tsdf_layer_->hasBlock(index)) {
        // The vertices should never lie outside allocated blocks.
        LOG(WARNING)
            << "Tried to color a mesh vertex outside allocated blocks.";
        return;
      }
      const TsdfBlock& neighbor_block = tsdf_layer_->getBlockByIndex(index);
      const TsdfVoxel& voxel = neighbor_block.getVoxelByCoordinates(vertex);
      voxblox::utils::getColorIfValid(voxel, config_.min_weight,
                                      &(mesh->colors[i]));
    }
  }
}

}  // namespace panoptic_mapping
