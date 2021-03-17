#ifndef PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/integrator/integrator_base.h"
#include "panoptic_mapping/integrator/projection_interpolators.h"

namespace panoptic_mapping {

/**
 * Allocate blocks based on the 3D points and project all visible blocks into
 * the image for updates.
 */
class ProjectiveIntegrator : public IntegratorBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Integration params.
    bool use_weight_dropoff = true;
    bool use_constant_weight = false;
    float max_weight = 1e5;
    std::string interpolation_method;  // nearest, bilinear, adaptive, semantic

    // Behavior
    bool foreign_rays_clear = true;  // observations of object B can clear
    // spcae in object A

    // System params.
    int integration_threads = std::thread::hardware_concurrency();

    // Camera settings.
    Camera::Config camera;
    std::string camera_namespace = "";

    Config() { setConfigName("ProjectiveIntegrator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit ProjectiveIntegrator(const Config& config);
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
      float* sdf, float* weight, bool* point_belongs_to_this_submap,
      InterpolatorBase* interpolator, const Point& p_C,
      const cv::Mat& color_image, const cv::Mat& id_image, int submap_id,
      float truncation_distance, float voxel_size,
      bool is_free_space_submap) const;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<IntegratorBase,
                                                    ProjectiveIntegrator>
      registration_;

  std::vector<std::unique_ptr<InterpolatorBase>>
      interpolators_;  // one for each thread.
  Camera camera_;

  // Cached data.
  Eigen::MatrixXf range_image_;
  float max_range_in_image_ = 0.f;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_
