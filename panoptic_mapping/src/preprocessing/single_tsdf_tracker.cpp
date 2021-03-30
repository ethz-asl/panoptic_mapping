#include "panoptic_mapping/preprocessing/single_tsdf_tracker.h"

#include <memory>
#include <unordered_set>
#include <utility>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, SingleTSDFTracker,
                                           std::shared_ptr<Globals>>
    SingleTSDFTracker::registration_("single_tsdf");

void SingleTSDFTracker::Config::checkParams() const {
  checkParamGT(voxels_per_side, 0, "voxels_per_side");
  checkParamGT(voxel_size, 0.f, "voxel_size");
}

void SingleTSDFTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("voxel_size", &voxel_size);
  setupParam("truncation_distance", &truncation_distance);
  setupParam("voxels_per_side", &voxels_per_side);
}

SingleTSDFTracker::SingleTSDFTracker(const Config& config,
                                     std::shared_ptr<Globals> globals)
    : config_(config.checkValid()), IDTrackerBase(std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  setRequiredInputs(
      {InputData::InputType::kColorImage, InputData::InputType::kDepthImage});
}

void SingleTSDFTracker::processInput(SubmapCollection* submaps,
                                     InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));

  // Check whether the map is already allocated.
  if (!is_setup_) {
    setup(submaps);
  }

  // Set the id data.
  input->setIdImage(cv::Mat(input->depthImage().rows, input->depthImage().cols,
                            CV_16SC1, map_id_));
}

void SingleTSDFTracker::setup(SubmapCollection* submaps) {
  // Check if there is a loaded map.
  if (submaps->size() > 0) {
    Submap* map = submaps->begin()->get();
    map->setIsActive(true);
    map_id_ = map->getID();
  } else {
    // Allocate the single map.
    Submap::Config config;
    config.voxels_per_side = config_.voxels_per_side;
    config.voxel_size = config_.voxel_size;
    config.truncation_distance = config_.truncation_distance;
    if (config.truncation_distance < 0.f) {
      config.truncation_distance *= -config.voxel_size;
    }
    Submap* new_submap = submaps->createSubmap(config);
    new_submap->setLabel(PanopticLabel::kBackground);
    map_id_ = new_submap->getID();
  }
  submaps->setActiveFreeSpaceSubmapID(map_id_);
  is_setup_ = true;
}

}  // namespace panoptic_mapping
