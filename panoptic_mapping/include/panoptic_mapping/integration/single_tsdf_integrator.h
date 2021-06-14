#ifndef PANOPTIC_MAPPING_INTEGRATION_SINGLE_TSDF_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATION_SINGLE_TSDF_INTEGRATOR_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/integration/projective_tsdf_integrator.h"

namespace panoptic_mapping {

/**
 * Only one submap to emulate a monolithic approach.
 */
class SingleTsdfIntegrator : public ProjectiveIntegrator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    ProjectiveIntegrator::Config pi_config;

    Config() { setConfigName("SingleTsdfIntegrator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  SingleTsdfIntegrator(const Config& config, std::shared_ptr<Globals> globals);
  ~SingleTsdfIntegrator() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  void allocateNewBlocks(Submap* map, InputData* input);

  virtual void updateBlock(Submap* submap, InterpolatorBase* interpolator,
                           const voxblox::BlockIndex& block_index,
                           const Transformation& T_C_S,
                           const cv::Mat& color_image,
                           const cv::Mat& id_image) const;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      TsdfIntegratorBase, SingleTsdfIntegrator, std::shared_ptr<Globals>>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATION_SINGLE_TSDF_INTEGRATOR_H_
