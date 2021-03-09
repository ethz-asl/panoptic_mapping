// NOTE(schmluk): This code was heavily taken from voxblox, which based the
// class on code from Open Chisel (copyright below).

// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef PANOPTIC_MAPPING_INTEGRATOR_MESH_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATOR_MESH_INTEGRATOR_H_

#include <algorithm>
#include <list>
#include <memory>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/mesh/mesh_layer.h>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {

/**
 * Integrates a TSDF layer to incrementally update a mesh layer using marching
 * cubes. Can optionally supply a second layer specifying whether a voxel
 * belongs to the submap.
 */
class MeshIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config : public config_utilities::Config<Config> {
    bool use_color = true;
    float min_weight = 1e-4;
    int required_belonging_corners = 4;  // Include voxels that have this many
    // corners belonging to the submap to still be fully meshed [0-8]
    int integrator_threads = std::thread::hardware_concurrency();

    Config() { setConfigName("MeshIntegrator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  MeshIntegrator(const MeshIntegrator::Config& config,
                 std::shared_ptr<TsdfLayer> tsdf_layer,
                 std::shared_ptr<MeshLayer> mesh_layer,
                 std::shared_ptr<ClassLayer> class_layer,
                 float truncation_distance = 0.f);

  MeshIntegrator(const MeshIntegrator::Config& config,
                 std::shared_ptr<TsdfLayer> tsdf_layer,
                 std::shared_ptr<MeshLayer> mesh_layer);

  /// Generates mesh from the tsdf layer.
  void generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag,
                    bool use_class_data = true);

 protected:
  void generateMeshBlocksFunction(
      const voxblox::BlockIndexList& all_tsdf_blocks, bool clear_updated_flag,
      voxblox::ThreadSafeIndex* index_getter);

  void updateMeshForBlock(const voxblox::BlockIndex& block_index);

  void extractBlockMesh(const TsdfBlock& tsdf_block,
                        const ClassBlock* class_block, voxblox::Mesh* mesh);

  void extractMeshInsideBlock(const TsdfBlock& tsdf_block,
                              const ClassBlock* class_block,
                              const voxblox::VoxelIndex& index,
                              const Point& coords,
                              voxblox::VertexIndex* next_mesh_index,
                              voxblox::Mesh* mesh);

  void extractMeshOnBorder(const TsdfBlock& tsdf_block,
                           const ClassBlock* class_block,
                           const voxblox::VoxelIndex& index,
                           const Point& coords,
                           voxblox::VertexIndex* next_mesh_index,
                           voxblox::Mesh* mesh);

  void updateMeshColor(const TsdfBlock& tsdf_block,
                       const ClassBlock* class_block, voxblox::Mesh* mesh);

 protected:
  const MeshIntegrator::Config config_;

  /**
   * Having both a const and a mutable pointer to the layer allows this
   * integrator to work both with a const layer (in case you don't want to clear
   * the updated flag) and mutable layer (in case you do want to clear the
   * updated flag).
   */
  std::shared_ptr<TsdfLayer> tsdf_layer_;
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<ClassLayer> class_layer_ = nullptr;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;
  bool use_class_layer_;
  FloatingPoint truncation_distance_;  // This only needs to be set if the class
  // layer is used, specifies distance assumed for foreign voxels.

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;

  // Cached index map.
  Eigen::Matrix<int, 3, 8> cube_index_offsets_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATOR_MESH_INTEGRATOR_H_