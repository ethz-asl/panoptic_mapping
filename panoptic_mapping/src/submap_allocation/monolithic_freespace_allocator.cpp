#include "panoptic_mapping/submap_allocation/monolithic_freespace_allocator.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<FreespaceAllocatorBase,
                                           MonolithicFreespaceAllocator>
    MonolithicFreespaceAllocator::registration_("monolithic");

void MonolithicFreespaceAllocator::Config::checkParams() const {
  checkParamConfig(submap);
}

void MonolithicFreespaceAllocator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("submap", &submap);
}

MonolithicFreespaceAllocator::MonolithicFreespaceAllocator(const Config& config,
                                                           bool print_config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
}

Submap* MonolithicFreespaceAllocator::allocateSubmap(SubmapCollection* submaps,
                                                     InputData* /*input*/) {
  if (submaps->getActiveFreeSpaceSubmapID() >= 0) {
    // Allocate one free space submap in the beginning.
    return nullptr;
  }

  // Otherwise create a new freespace submap.
  Submap* space_submap = submaps->createSubmap(config_.submap);
  space_submap->setLabel(PanopticLabel::kFreeSpace);
  space_submap->setInstanceID(-1);  // Will never appear in a seg image.
  space_submap->setName("FreeSpace");
  submaps->setActiveFreeSpaceSubmapID(space_submap->getID());
  return space_submap;
}

}  // namespace panoptic_mapping
