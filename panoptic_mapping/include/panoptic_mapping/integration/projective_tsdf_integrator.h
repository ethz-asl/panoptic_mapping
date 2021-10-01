#ifndef PANOPTIC_MAPPING_INTEGRATION_PROJECTIVE_TSDF_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATION_PROJECTIVE_TSDF_INTEGRATOR_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/integration/projection_interpolators.h"
#include "panoptic_mapping/integration/tsdf_integrator_base.h"

namespace panoptic_mapping {

/**
 * @brief Allocate blocks based on the 3D points and project all visible blocks
 * into the image for updates.
 * TODO(schmluk): properly use validity image?
 */
class ProjectiveIntegrator : public TsdfIntegratorBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // If true, drop off the weight behind the surface crossing.
    bool use_weight_dropoff = true;

    // Distance in meters where the weight dropp off reaches zero. Negative
    // values  are multiples of the voxel size.
    float weight_dropoff_epsilon = -1.f;

    // If true, use unitary (w=1) weights to update the TSDF. Otherwise use
    // weights as a function of the squared depth to approximate typical RGBD
    // sensor confidence.
    bool use_constant_weight = false;

    // Maximum weight used for TSDF updates. High max weight keeps information
    // longer in memory, low max weight favors rapid updates.
    float max_weight = 1e5;

    // Interpolation method to be used when interpolating values in the images.
    // Supported are {nearest, bilinear, adaptive}.
    std::string interpolation_method = "adaptive";

    // If true, rays that don't belong to the submap ID are treated as clearing
    // rays.
    bool foreign_rays_clear = true;

    // If false, allocate blocks where points fall directly into. If true, also
    // allocate neighboring blocks if the voxels lie on the boundary of the
    // block.
    bool allocate_neighboring_blocks = false;

    // If true, overwrite voxels that have a distance update larger than 0.05 m.
    bool use_longterm_fusion = false;

    // Number of threads used to perform integration. Integration is
    // submap-parallel.
    int integration_threads = std::thread::hardware_concurrency();

    Config() { setConfigName("ProjectiveTsdfIntegrator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  ProjectiveIntegrator(const Config& config, std::shared_ptr<Globals> globals,
                       bool print_config = true);
  ~ProjectiveIntegrator() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  /**
   * @brief Allocate all new blocks in all submaps.
   *
   * @param submaps Submap collection to allocate data in.
   * @param input Input measurements based on which blocks are allocated.
   */
  virtual void allocateNewBlocks(SubmapCollection* submaps,
                                 const InputData& input);

  virtual void updateSubmap(Submap* submap, InterpolatorBase* interpolator,
                            const voxblox::BlockIndexList& block_indices,
                            const InputData& input) const;

  virtual void updateBlock(Submap* submap, InterpolatorBase* interpolator,
                           const voxblox::BlockIndex& block_index,
                           const Transformation& T_C_S,
                           const InputData& input) const;

  /**
   * @brief Update a specific voxel based on the input.
   *
   * @param interpolator Projection interpolator to use. Interpolators are not
   thread safe.
   * @param voxel Tsdf voxel to be updated.
   * @param p_C Position of the voxel center in camera frame in meters.
   * @param input Input data used for the update.
   * @param submap_id SubmapID of the owning submap.
   * @param is_free_space_submap Whether the voxel belongs to a freespace map.
   * @param truncation_distance Truncation distance to be used.
   * @param voxel_size Voxel size of the TSDF layer.
   * @param class_voxel Optional: class voxel to be updated.

   * @return True if the voxel was updated.
   */
  virtual bool updateVoxel(InterpolatorBase* interpolator, TsdfVoxel* voxel,
                           const Point& p_C, const InputData& input,
                           const int submap_id, const bool is_free_space_submap,
                           const float truncation_distance,
                           const float voxel_size,
                           ClassVoxel* class_voxel = nullptr) const;

  /**
   * @brief Sets up the interpolator and computes the signed distance.
   *
   * @param p_C Voxel center in camera frame in meters.
   * @param interpolator Interpolator to setup and use.
   * @return Whether the voxel is valid to continue processing.
   */
  virtual bool computeSignedDistance(const Point& p_C,
                                     InterpolatorBase* interpolator,
                                     float* sdf) const;

  /**
   * @brief Compute the measurement weight for a given voxel based on the
   * parameters set in the config.
   *
   * @param p_C Voxel center in camera frame in meters.
   * @param voxel_size The voxel size in meters.
   * @param truncation_distance The truncation distance in meters.
   * @param sdf The signed distance used for this update.
   * @return The measurement weight.
   */
  virtual float computeWeight(const Point& p_C, const float voxel_size,
                              const float truncation_distance,
                              const float sdf) const;

  /**
   * @brief Update the values of a voxel in a weighted averaging fashion.
   *
   * @param voxel The voxel to be updated. The pointer is not checked for
   * validity.
   * @param sdf The signed distance measurement to be fused, already truncated
   * to the truncation distance.
   * @param weight The measurement weight to be used for the update.
   * @param color Optional pointer to a color to be fused.
   */
  virtual void updateVoxelValues(TsdfVoxel* voxel, const float sdf,
                                 const float weight,
                                 const Color* color = nullptr) const;

  // Cached data.
  Eigen::MatrixXf range_image_;
  float max_range_in_image_ = 0.f;
  const Camera::Config* cam_config_;
  std::vector<std::unique_ptr<InterpolatorBase>>
      interpolators_;  // one for each thread.

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      TsdfIntegratorBase, ProjectiveIntegrator, std::shared_ptr<Globals>>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATION_PROJECTIVE_TSDF_INTEGRATOR_H_
