#include "panoptic_mapping/core/planning_interface.h"

#include <memory>
#include <utility>

namespace panoptic_mapping {

PlanningInterface::PlanningInterface(
    std::shared_ptr<const SubmapCollection> submaps)
    : submaps_(std::move(submaps)) {}

}  // namespace panoptic_mapping
