#ifndef PANOPTIC_MAPPING_SUBMAP_ALLOCATION_MONOLITHIC_FREESPACE_ALLOCATOR_H_
#define PANOPTIC_MAPPING_SUBMAP_ALLOCATION_MONOLITHIC_FREESPACE_ALLOCATOR_H_

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/labels/label_handler_base.h"
#include "panoptic_mapping/submap_allocation/freespace_allocator_base.h"

namespace panoptic_mapping {

/**
 * @brief This submap allocator creates a monolithic submap to cover all
 * freespace.
 */
class MonolithicFreespaceAllocator : public FreespaceAllocatorBase {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // All values for the submap config except the voxel sizes. Use negative
    // truncation distance to make them multiples of the voxelsizes.
    Submap::Config submap;

    Config() { setConfigName("MonolithicFreespaceAllocator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Construction.
  explicit MonolithicFreespaceAllocator(const Config& config,
                                        bool print_config = true);
  ~MonolithicFreespaceAllocator() override = default;

  Submap* allocateSubmap(SubmapCollection* submaps,
                         InputData* /* input */) override;

 private:
  static config_utilities::Factory::RegistrationRos<
      FreespaceAllocatorBase, MonolithicFreespaceAllocator>
      registration_;
  const Config config_;
};
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_SUBMAP_ALLOCATION_MONOLITHIC_FREESPACE_ALLOCATOR_H_
