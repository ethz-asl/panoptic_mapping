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
#include "panoptic_mapping/map/classification/uncertainty.h"

namespace panoptic_mapping {

/**
 * @brief Integrator that integrates all data into a single submap to emulate a
 * monolithic approach. Combine this module with the SingleTsdfIDTracker.
 */
class SingleTsdfIntegrator : public ProjectiveIntegrator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Standard integrator params.
    ProjectiveIntegrator::Config projective_integrator;

    // If true require a color image and update voxel colors.
    bool use_color = true;

    // If true require a segmentation image and integrate it into a class layer.
    bool use_segmentation = true;

    // If true require an uncertainty image and integrate it into an class layer
    // of type 'UncertaintyLayer'.
    bool use_uncertainty = false;

    // Decay rate in [0, 1] used to update uncertainty voxels. Only used if
    // 'use_uncertainty' is true.
    float uncertainty_decay_rate = 0.5f;

    Config() { setConfigName("SingleTsdfIntegrator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  SingleTsdfIntegrator(const Config& config, std::shared_ptr<Globals> globals);
  ~SingleTsdfIntegrator() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  // Override methods specific to the single TSDF update.
  void allocateNewBlocks(Submap* map, InputData* input);

  void updateBlock(Submap* submap, InterpolatorBase* interpolator,
                   const voxblox::BlockIndex& block_index,
                   const Transformation& T_C_S,
                   const InputData& input) const override;

  bool updateVoxel(InterpolatorBase* interpolator, TsdfVoxel* voxel,
                   const Point& p_C, const InputData& input,
                   const int submap_id, const bool is_free_space_submap,
                   const float truncation_distance, const float voxel_size,
                   ClassVoxel* class_voxel = nullptr) const override;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      TsdfIntegratorBase, SingleTsdfIntegrator, std::shared_ptr<Globals>>
      registration_;

  void updateClassVoxel(InterpolatorBase* interpolator, const InputData& input,
                        ClassVoxel* class_voxel) const;

  void updateUncertaintyVoxel(InterpolatorBase* interpolator,
                              const InputData& input,
                              UncertaintyVoxel* class_voxel) const;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATION_SINGLE_TSDF_INTEGRATOR_H_
