#include "panoptic_mapping/preprocessing/detectron_id_tracker.h"

#include <memory>
#include <unordered_map>
#include <utility>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, DetectronIDTracker,
                                           std::shared_ptr<Globals>>
    DetectronIDTracker::registration_("detectron");

void DetectronIDTracker::Config::checkParams() const {
  checkParamConfig(projective_id_tracker);
  checkParamConfig(edge_refiner);
}

void DetectronIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("use_edge_refinement", &use_edge_refinement);
  setupParam("use_class_layer", &use_class_layer);
  setupParam("projective_id_tracker", &projective_id_tracker);
  setupParam("edge_refiner", &edge_refiner);
}

DetectronIDTracker::DetectronIDTracker(const Config& config,
                                       std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIDTracker(config.projective_id_tracker, std::move(globals)),
      edge_refiner_(config_.edge_refiner) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  addRequiredInput(InputData::InputType::kDetectronLabels);
  if (config_.use_edge_refinement) {
    edge_refiner_.setup(globals_->camera()->getConfig());
  }
}

void DetectronIDTracker::processInput(SubmapCollection* submaps,
                                      InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Cache the input labels for submap allocation.
  labels_ = &(input->detectronLabels());

  // If requested refine the edges of the predictions.
  if (config_.use_edge_refinement) {
    if (visualizationIsOn()) {
      visualize(renderer_.colorIdImage(*(input->idImage())), "unrefined_input");
    }
    edge_refiner_.refinePrediction(input->depthImage(), input->vertexMap(),
                                   input->idImage());
    if (visualizationIsOn()) {
      cv::Mat normals;
      edge_refiner_.getNormalMap().convertTo(normals, CV_8UC3, 127.5f, 127.5f);
      visualize(normals, "normals");
    }
  }

  // Track the predicted (and refined) ids.
  ProjectiveIDTracker::processInput(submaps, input);
}

int DetectronIDTracker::allocateSubmap(int detectron_id,
                                       SubmapCollection* submaps) {
  // Check whether the instance code is known.
  auto it = labels_->find(detectron_id);
  PanopticLabel pan_label;
  if (it == labels_->end()) {
    return -1;
  } else {
    if (it->second.is_thing) {
      pan_label = PanopticLabel::kInstance;
    } else {
      pan_label = PanopticLabel::kBackground;
    }
  }

  // Allocate new submap.
  Submap::Config cfg;
  cfg.voxels_per_side = config_.projective_id_tracker.voxels_per_side;
  switch (pan_label) {
    case PanopticLabel::kInstance: {
      cfg.voxel_size = config_.projective_id_tracker.instance_voxel_size;
      break;
    }
    case PanopticLabel::kBackground: {
      cfg.voxel_size = config_.projective_id_tracker.background_voxel_size;
      break;
    }
  }
  Submap* new_submap = submaps->createSubmap(cfg);
  new_submap->setLabel(pan_label);
  new_submap->setClassID(it->second.category_id);
  new_submap->getClassLayerPtr() =
      std::make_shared<ClassLayer>(cfg.voxel_size, cfg.voxels_per_side);
  new_submap->setName(globals_->labelHandler()->getName(detectron_id));
  return new_submap->getID();
}

}  // namespace panoptic_mapping
