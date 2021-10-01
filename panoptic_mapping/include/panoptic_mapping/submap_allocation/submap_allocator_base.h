#ifndef PANOPTIC_MAPPING_SUBMAP_ALLOCATION_SUBMAP_ALLOCATOR_BASE_H_
#define PANOPTIC_MAPPING_SUBMAP_ALLOCATION_SUBMAP_ALLOCATOR_BASE_H_

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/input_data_user.h"
#include "panoptic_mapping/labels/label_entry.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * @brief Interface for submap allocators. Allocators are called by the ID
 * Trackers to create new submaps if suitable.
 */
class SubmapAllocatorBase : public InputDataUser {
 public:
  SubmapAllocatorBase() = default;
  ~SubmapAllocatorBase() override = default;

  // Interface.
  /**
   * @brief Allocate a new submap based on the provided input data and label.
   *
   * @param submaps Collection to allocate the submap in.
   * @param input The input data based on which the submap should be allocated.
   * @param input_id The id in the id image associated with the submap to be
   * allocated.
   * @param label The label data to be used to create this submap.
   * @return Pointer to the newly allocated submap, nullptr if submap allocation
   * failed.
   */
  virtual Submap* allocateSubmap(SubmapCollection* submaps, InputData* input,
                                 int input_id, const LabelEntry& label) = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_SUBMAP_ALLOCATION_SUBMAP_ALLOCATOR_BASE_H_
