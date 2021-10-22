#ifndef PANOPTIC_MAPPING_INTEGRATION_CLASS_PROJECTIVE_TSDF_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATION_CLASS_PROJECTIVE_TSDF_INTEGRATOR_H_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/integration/projection_interpolators.h"
#include "panoptic_mapping/integration/projective_tsdf_integrator.h"
#include "panoptic_mapping/integration/tsdf_integrator_base.h"

namespace panoptic_mapping {

/**
 * @brief Allocate blocks based on the 3D points and project all visible blocks
 * into the image for updates. Also update a separate class layer to estimate
 * which voxels belong to the current submap.
 */
class ClassProjectiveIntegrator : public ProjectiveIntegrator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    bool use_binary_classification = false;    // false: use a counter per id.
    bool use_instance_classification = false;  // false: use class id.
    bool update_only_tracked_submaps = true;

    // Integration params.
    ProjectiveIntegrator::Config pi_config;

    Config() { setConfigName("ClassProjectiveTsdfIntegrator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  ClassProjectiveIntegrator(const Config& config,
                            std::shared_ptr<Globals> globals);
  ~ClassProjectiveIntegrator() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  void updateBlock(Submap* submap, InterpolatorBase* interpolator,
                   const voxblox::BlockIndex& block_index,
                   const Transformation& T_C_S,
                   const InputData& input) const override;

  bool updateVoxel(InterpolatorBase* interpolator, TsdfVoxel* voxel,
                   const Point& p_C, const InputData& input,
                   const int submap_id, const bool is_free_space_submap,
                   const float truncation_distance, const float voxel_size,
                   ClassVoxelType* class_voxel = nullptr) const override;

  void updateClassVoxel(InterpolatorBase* interpolator, ClassVoxel* voxel,
                        const InputData& input, const int submap_id) const;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      TsdfIntegratorBase, ClassProjectiveIntegrator, std::shared_ptr<Globals>>
      registration_;

  // Cached data.
  std::unordered_map<int, int> id_to_class_;
  size_t num_classes_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATION_CLASS_PROJECTIVE_TSDF_INTEGRATOR_H_
