#ifndef PANOPTIC_MAPPING_SUBMAP_ALLOCATION_SEMANTIC_SUBMAP_ALLOCATOR_H_
#define PANOPTIC_MAPPING_SUBMAP_ALLOCATION_SEMANTIC_SUBMAP_ALLOCATOR_H_

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/labels/label_entry.h"
#include "panoptic_mapping/submap_allocation/submap_allocator_base.h"

namespace panoptic_mapping {

/**
 * @brief This submap allocator chooses a voxel size based on the semantic label
 * of submaps.
 */
class SemanticSubmapAllocator : public SubmapAllocatorBase {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // All values for the submap config except the voxel sizes and truncation
    // distance, these are individually set below.
    Submap::Config submap;

    // Voxel size for each label in meters.
    float small_instance_voxel_size = 0.03;
    float medium_instance_voxel_size = 0.05;
    float large_instance_voxel_size = 0.07;
    float background_voxel_size = 0.1;
    float unknown_voxel_size = 0.1;

    // Truncation distance for all labels in meters. This overwrites the
    // submap. Negative values are multiples of the voxel size.
    float truncation_distance = -2.f;

    Config() { setConfigName("SemanticSubmapAllocator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Construction.
  explicit SemanticSubmapAllocator(const Config& config,
                                   bool print_config = true);
  ~SemanticSubmapAllocator() override = default;

  Submap* allocateSubmap(SubmapCollection* submaps, InputData* /* input */,
                         int /* input_id */, const LabelEntry& label) override;

 private:
  static config_utilities::Factory::RegistrationRos<SubmapAllocatorBase,
                                                    SemanticSubmapAllocator>
      registration_;
  const Config config_;
};
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_SUBMAP_ALLOCATION_SEMANTIC_SUBMAP_ALLOCATOR_H_
