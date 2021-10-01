#include "panoptic_mapping/submap_allocation/null_submap_allocator.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<SubmapAllocatorBase,
                                           NullSubmapAllocator>
    NullSubmapAllocator::registration_("null");

void NullSubmapAllocator::Config::setupParamsAndPrinting() {
  printText("The null submap allocator does not allocate any submaps.");
}

NullSubmapAllocator::NullSubmapAllocator(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

config_utilities::Factory::RegistrationRos<FreespaceAllocatorBase,
                                           NullFreespaceAllocator>
    NullFreespaceAllocator::registration_("null");

void NullFreespaceAllocator::Config::setupParamsAndPrinting() {
  printText("The null free space allocator does not allocate any submaps.");
}

NullFreespaceAllocator::NullFreespaceAllocator(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

}  // namespace panoptic_mapping
