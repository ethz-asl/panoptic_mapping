#include "panoptic_mapping/submap_allocation/semantic_submap_allocator.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<SubmapAllocatorBase,
                                           SemanticSubmapAllocator>
    SemanticSubmapAllocator::registration_("semantic");

void SemanticSubmapAllocator::Config::checkParams() const {
  // NOTE(schmluk): the submap config is not checked for validity to not
  // initialize the truncation distance.
  checkParamGT(small_instance_voxel_size, 0.f, "small_instance_voxel_size");
  checkParamGT(medium_instance_voxel_size, 0.f, "medium_instance_voxel_size");
  checkParamGT(large_instance_voxel_size, 0.f, "large_instance_voxel_size");
  checkParamGT(background_voxel_size, 0.f, "background_voxel_size");
  checkParamGT(unknown_voxel_size, 0.f, "unknown_voxel_size");
}

void SemanticSubmapAllocator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("submap_config", &submap_config);
  setupParam("small_instance_voxel_size", &small_instance_voxel_size);
  setupParam("medium_instance_voxel_size", &medium_instance_voxel_size);
  setupParam("large_instance_voxel_size", &large_instance_voxel_size);
  setupParam("background_voxel_size", &background_voxel_size);
  setupParam("unknown_voxel_size", &unknown_voxel_size);
}

SemanticSubmapAllocator::SemanticSubmapAllocator(const Config& config,
                                                 bool print_config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
}

Submap* SemanticSubmapAllocator::allocateSubmap(
    SubmapCollection* submaps, InputData* /* input */, int input_id,
    const LabelHandler::LabelEntry& label) {
  Submap::Config config = config_.submap_config;

  // Setup the voxel size.
  switch (label.label) {
    case PanopticLabel::kInstance: {
      if (strcasecmp(label.size.c_str(), "L")) {
        config.voxel_size = config_.large_instance_voxel_size;
      } else if (strcasecmp(label.size.c_str(), "S")) {
        config.voxel_size = config_.small_instance_voxel_size;
      } else {
        config.voxel_size = config_.medium_instance_voxel_size;
      }
      break;
    }
    case PanopticLabel::kBackground: {
      config.voxel_size = config_.background_voxel_size;
      break;
    }
    default: {
      config.voxel_size = config_.unknown_voxel_size;
      break;
    }
  }

  // Check the truncation distance is met.
  if (config.truncation_distance < 0.f) {
    config.truncation_distance *= -config.voxel_size;
  }

  // Create the submap.
  Submap* new_submap = submaps->createSubmap(config);
  new_submap->setClassID(label.class_id);
  new_submap->setLabel(label.label);
  new_submap->setName(label.name);

  return new_submap;
}

}  // namespace panoptic_mapping
