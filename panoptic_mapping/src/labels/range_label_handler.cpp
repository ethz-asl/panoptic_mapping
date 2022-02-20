#include "panoptic_mapping/labels/range_label_handler.h"

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<LabelHandlerBase, RangeLabelHandler>
    RangeLabelHandler::registration_("range");

void RangeLabelHandler::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("num_labels", &num_labels);
}

void RangeLabelHandler::Config::checkParams() const {
  checkParamGT(num_labels, 0, "num_labels");
}

RangeLabelHandler::RangeLabelHandler(const Config& config, bool print_config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
  // Setup the labels from range.
  initialiseLabels();
}

void RangeLabelHandler::initialiseLabels() {
  for (int i; i < config_.num_labels; i++) {
    LabelEntry label;
    label.segmentation_id = i;
    label.class_id = i;
    label.label = PanopticLabel::kBackground;
    label.color = voxblox::rainbowColorMap(((float)i) / config_.num_labels);
    labels_[i] = std::make_unique<LabelEntry>(label);
  }
}

}  // namespace panoptic_mapping
