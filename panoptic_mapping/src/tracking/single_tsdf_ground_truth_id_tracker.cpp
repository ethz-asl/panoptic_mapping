#include "panoptic_mapping/tracking/single_tsdf_ground_truth_id_tracker.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<
    IDTrackerBase, SingleTSDFGroundTruthIDTracker, std::shared_ptr<Globals>>
    SingleTSDFGroundTruthIDTracker::registration_("single_tsdf_ground_truth");

void SingleTSDFGroundTruthIDTracker::Config::checkParams() const {
  checkParamConfig(submap);
  checkParamConfig(renderer);
}

void SingleTSDFGroundTruthIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("submap", &submap);
  setupParam("renderer", &renderer);
}

SingleTSDFGroundTruthIDTracker::SingleTSDFGroundTruthIDTracker(
    const Config& config, std::shared_ptr<Globals> globals)
    : IDTrackerBase(std::move(globals)),
      config_(config.checkValid()),
      renderer_(config.renderer, globals_->camera()->getConfig(), false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  addRequiredInput(InputData::InputType::kColorImage);
  addRequiredInput(InputData::InputType::kDepthImage);
  addRequiredInput(InputData::InputType::kSegmentationImage);
}

void SingleTSDFGroundTruthIDTracker::processInput(SubmapCollection* submaps,
                                                  InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);

  CHECK(inputIsValid(*input));

  // Check whether the map is already allocated.
  if (!is_setup_) {
    setup(submaps);
  }

  // Visualization
  cv::Mat input_vis;
  if (visualizationIsOn()) {
    input_vis = renderer_.colorIdImage(input->idImage());
  }

  std::unordered_set<int> parsed_ids;
  // Parse input id image
  for (auto it = input->idImagePtr()->begin<int>();
       it != input->idImagePtr()->end<int>(); ++it) {
    // Skip the ignore label
    if (*it == 0) {
      continue;
    }

    // If this id has already been parsed, skip it
    if (parsed_ids.find(*it) != parsed_ids.end()) {
      continue;
    }

    // Sanity check the id - it should be in the ground truth labels
    if (!globals_->labelHandler()->segmentationIdExists(*it)) {
      LOG(FATAL) << "Unexpected id " << *it << " in segmentation image!\n";
    }

    // Update tracked instance info for instance classes
    if (globals_->labelHandler()->isInstanceClass(*it)) {
      LOG_IF(INFO, config_.verbosity >= 3)
          << "Updating instance info for instance of class "
          << globals_->labelHandler()->getName(*it);
      // Add to tracked instances if not already there
      SegmentInfo instance_segment_info;
      instance_segment_info.class_id =
          globals_->labelHandler()->getClassID(*it);
      instance_segment_info.instance_score = 0.99;
      instance_segment_info.class_score = 0.99;
      submaps->updateTrackedInstanceInfo(*it, instance_segment_info, 1.f);
    }
    
    parsed_ids.insert(*it);
  }

  // Publish Visualization if requested.
  if (visualizationIsOn()) {
    cv::Mat tracked_vis = renderer_.colorIdImage(input->idImage());
    visualize(input_vis, "input");
    visualize(rendered_vis_, "rendered");
    visualize(input->colorImage(), "color");
    visualize(tracked_vis, "tracked");
  }
}

void SingleTSDFGroundTruthIDTracker::setup(SubmapCollection* submaps) {
  // Check if there is a loaded map.
  if (submaps->size() > 0) {
    Submap& map = *(submaps->begin());
    if (map.getConfig().voxel_size != config_.submap.voxel_size ||
        map.getConfig().voxels_per_side != config_.submap.voxels_per_side ||
        map.getConfig().truncation_distance !=
            config_.submap.truncation_distance ||
        map.getConfig().useClassLayer() != config_.submap.useClassLayer()) {
      LOG(WARNING)
          << "Loaded submap config does not match the specified config.";
    }
    map.setIsActive(true);
    map_id_ = map.getID();
  } else {
    // Allocate the single map.
    Submap* new_submap = submaps->createSubmap(config_.submap);
    new_submap->setLabel(PanopticLabel::kBackground);
    map_id_ = new_submap->getID();
  }
  submaps->setActiveFreeSpaceSubmapID(map_id_);
  is_setup_ = true;
}

}  // namespace panoptic_mapping
