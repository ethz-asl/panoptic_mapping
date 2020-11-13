#ifndef PANOPTIC_MAPPING_CORE_PLANNING_INTERFACE_H_
#define PANOPTIC_MAPPING_CORE_PLANNING_INTERFACE_H_

#include <memory>

#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/submap_collection.h"

namespace panoptic_mapping {

/**
 * This class implements high level interfaces for lookups on the submap
 * collection.
 */
class PlanningInterface {
 public:
  explicit PlanningInterface(std::shared_ptr<const SubmapCollection> submaps);

  enum class VoxelState {
    kUnknown = 0,
    kKnownFree,
    kKnownOccupied,
    kExpectedFree,
    kExpectedOccupied
  };

  // Access.
  const SubmapCollection& getSubmapCollection() const { return *submaps_; }

  // Lookups.
  bool isObserved(const Point& position,
                  bool include_inactive_maps = true) const;
  VoxelState getVoxelState(const Point& position) const;

 private:
  std::shared_ptr<const SubmapCollection> submaps_;
  static constexpr float kObservedMinWeight_ = 1e-6;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_PLANNING_INTERFACE_H_
