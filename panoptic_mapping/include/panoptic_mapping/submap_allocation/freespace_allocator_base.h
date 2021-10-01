#ifndef PANOPTIC_MAPPING_SUBMAP_ALLOCATION_FREESPACE_ALLOCATOR_BASE_H_
#define PANOPTIC_MAPPING_SUBMAP_ALLOCATION_FREESPACE_ALLOCATOR_BASE_H_

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/input_data_user.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * @brief Interface for freespace allocators. Allocators are called by the ID
 * Trackers to create new submaps if suitable.
 */
class FreespaceAllocatorBase : public InputDataUser {
 public:
  FreespaceAllocatorBase() = default;
  ~FreespaceAllocatorBase() override = default;

  // Interface.
  /**
   * @brief Allocate a new submap based on the provided input data.
   *
   * @param submaps Collection to allocate the submap in.
   * @param input The input data based on which the submap should be allocated.
   * @return Pointer to the newly allocated submap, nullptr if submap allocation
   * failed.
   */
  virtual Submap* allocateSubmap(SubmapCollection* submaps,
                                 InputData* input) = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_SUBMAP_ALLOCATION_FREESPACE_ALLOCATOR_BASE_H_
