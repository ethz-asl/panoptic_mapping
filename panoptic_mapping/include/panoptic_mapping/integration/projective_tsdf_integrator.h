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

    // Integration params.
    bool use_weight_dropoff = true;
    bool use_constant_weight = false;
    float max_weight = 1e5;
    std::string interpolation_method;  // nearest, bilinear, adaptive

    // Behavior
    bool foreign_rays_clear = true;  // observations of object B can clear
    // spcae in object A

    // System params.
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
  void allocateNewBlocks(SubmapCollection* submaps, InputData* input);

  void updateSubmap(Submap* submap, InterpolatorBase* interpolator,
                    const voxblox::BlockIndexList& block_indices,
                    const Transformation& T_M_C, const cv::Mat& color_image,
                    const cv::Mat& id_image) const;

  virtual void updateBlock(Submap* submap, InterpolatorBase* interpolator,
                           const voxblox::BlockIndex& block_index,
                           const Transformation& T_C_S,
                           const cv::Mat& color_image,
                           const cv::Mat& id_image) const;

  bool computeVoxelDistanceAndWeight(
      float* sdf, float* weight, int* id, InterpolatorBase* interpolator,
      const Point& p_C, const cv::Mat& color_image, const cv::Mat& id_image,
      int submap_id, float truncation_distance, float voxel_size,
      bool is_free_space_submap) const;

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
