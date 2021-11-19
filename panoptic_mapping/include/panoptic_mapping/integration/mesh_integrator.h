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
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef PANOPTIC_MAPPING_INTEGRATION_MESH_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATION_MESH_INTEGRATOR_H_

#include <algorithm>
#include <list>
#include <memory>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/mesh/mesh_layer.h>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/class_layer.h"

namespace panoptic_mapping {

/**
 * @brief Integrates a TSDF layer to incrementally update a mesh layer using
 * marching cubes. Can optionally supply a second layer specifying whether a
 * voxel belongs to the submap.
 */
class MeshIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Config : public config_utilities::Config<Config> {
    // Whether to use color information from the TSDF.
    bool use_color = true;

    // Minimum TSDF weight required to mesh a voxel.
    float min_weight = 1e-6;

    // If classification is used, how many of the corners of a mesh cube need to
    // labeled as belonging to the submap to still be meshed. Values are [0-8].
    int required_belonging_corners = 4;

    // Number of threads used to mesh a layer in parallel.
    int integrator_threads = std::thread::hardware_concurrency();

    // If true, voxels not belonging to this submap will be set to the
    // truncation distance.
    bool clear_foreign_voxels = false;

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

  // Generates the mesh from the tsdf layer.
  void generateMesh(bool only_mesh_updated_blocks = true,
                    bool clear_updated_flag = true,
                    bool use_class_data = false);

 protected:
  void generateMeshBlocksFunction(
      const voxblox::BlockIndexList& all_tsdf_blocks, bool clear_updated_flag,
      voxblox::ThreadSafeIndex* index_getter);

  bool updateMeshForBlock(const voxblox::BlockIndex& block_index);

  void extractBlockMesh(const TsdfBlock& tsdf_block,
                        const ClassBlock::ConstPtr& class_block,
                        voxblox::Mesh* mesh);

  void extractMeshInsideBlock(const TsdfBlock& tsdf_block,
                              const ClassBlock::ConstPtr& class_block,
                              const voxblox::VoxelIndex& index,
                              const Point& coords,
                              voxblox::VertexIndex* next_mesh_index,
                              voxblox::Mesh* mesh);

  void extractMeshOnBorder(const TsdfBlock& tsdf_block,
                           const ClassBlock::ConstPtr& class_block,
                           const voxblox::VoxelIndex& index,
                           const Point& coords,
                           voxblox::VertexIndex* next_mesh_index,
                           voxblox::Mesh* mesh);

  void updateMeshColor(const TsdfBlock& tsdf_block,
                       const ClassBlock::ConstPtr& class_block,
                       voxblox::Mesh* mesh);

 protected:
  const MeshIntegrator::Config config_;

  std::shared_ptr<TsdfLayer> tsdf_layer_;
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<ClassLayer> class_layer_;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  bool use_class_layer_;
  FloatingPoint truncation_distance_;  // This only needs to be set if the class
  // layer is used, specifies distance assumed for foreign voxels.

  // Cached index map.
  Eigen::Matrix<int, 3, 8> cube_index_offsets_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATION_MESH_INTEGRATOR_H_
