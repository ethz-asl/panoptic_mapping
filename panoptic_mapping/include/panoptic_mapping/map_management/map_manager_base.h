#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_BASE_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_BASE_H_

#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * @brief Interface for a high level class that wraps all map management actions
 * and tools.
 */
class MapManagerBase {
 public:
  MapManagerBase() = default;
  virtual ~MapManagerBase() = default;

  // Perform all actions when with specified timings.
  virtual void tick(SubmapCollection* submaps) = 0;
  virtual void finishMapping(SubmapCollection* submaps) = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_BASE_H_
