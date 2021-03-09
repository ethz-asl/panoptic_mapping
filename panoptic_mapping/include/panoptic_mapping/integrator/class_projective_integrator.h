#ifndef PANOPTIC_MAPPING_INTEGRATOR_CLASS_PROJECTIVE_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATOR_CLASS_PROJECTIVE_INTEGRATOR_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/integrator/integrator_base.h"
#include "panoptic_mapping/integrator/projection_interpolators.h"
#include "panoptic_mapping/integrator/projective_integrator.h"

namespace panoptic_mapping {

/**
 * Allocate blocks based on the 3D points and project all visible blocks into
 * the image for updates.
 */
class ClassProjectiveIntegrator : public ProjectiveIntegrator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Integration params.
    ProjectiveIntegrator::Config pi_config;

    Config() { setConfigName("ClassProjectiveIntegrator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit ClassProjectiveIntegrator(const Config& config);
  ~ClassProjectiveIntegrator() override = default;

 protected:
  void updateBlock(Submap* submap, InterpolatorBase* interpolator,
                   const voxblox::BlockIndex& block_index,
                   const Transformation& T_C_S, const cv::Mat& color_image,
                   const cv::Mat& id_image) const override;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<IntegratorBase,
                                                    ClassProjectiveIntegrator>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATOR_CLASS_PROJECTIVE_INTEGRATOR_H_
