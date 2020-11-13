#ifndef PANOPTIC_MAPPING_CORE_PLANNING_INTERFACE_H_
#define PANOPTIC_MAPPING_CORE_PLANNING_INTERFACE_H_

#include <memory>

#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/submap_collection.h"

namespace panoptic_mapping {

/**
 * This class implements high level lookups on the submap collection.
 */
class PlanningInterface {
 public:
  explicit PlanningInterface(std::shared_ptr<const SubmapCollection> submaps);

  // Access.
  const SubmapCollection& getSubmapCollection() const { return *submaps_; }

 private:
  std::shared_ptr<const SubmapCollection> submaps_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_PLANNING_INTERFACE_H_
