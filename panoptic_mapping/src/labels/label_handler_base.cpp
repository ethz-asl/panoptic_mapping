#include "panoptic_mapping/labels/label_handler_base.h"

#include <string>

namespace panoptic_mapping {

bool LabelHandlerBase::segmentationIdExists(int segmentation_id) const {
  return labels_.find(segmentation_id) != labels_.end();
}

int LabelHandlerBase::getClassID(int segmentation_id) const {
  return labels_.at(segmentation_id)->class_id;
}

bool LabelHandlerBase::isBackgroundClass(int segmentation_id) const {
  return labels_.at(segmentation_id)->label == PanopticLabel::kBackground;
}

bool LabelHandlerBase::isInstanceClass(int segmentation_id) const {
  return labels_.at(segmentation_id)->label == PanopticLabel::kInstance;
}

bool LabelHandlerBase::isUnknownClass(int segmentation_id) const {
  return labels_.at(segmentation_id)->label == PanopticLabel::kUnknown;
}

bool LabelHandlerBase::isSpaceClass(int segmentation_id) const {
  return labels_.at(segmentation_id)->label == PanopticLabel::kFreeSpace;
}

PanopticLabel LabelHandlerBase::getPanopticLabel(int segmentation_id) const {
  return labels_.at(segmentation_id)->label;
}

const voxblox::Color& LabelHandlerBase::getColor(int segmentation_id) const {
  return labels_.at(segmentation_id)->color;
}

const std::string& LabelHandlerBase::getName(int segmentation_id) const {
  return labels_.at(segmentation_id)->name;
}

const LabelEntry& LabelHandlerBase::getLabelEntry(int segmentation_id) const {
  return *labels_.at(segmentation_id);
}

bool LabelHandlerBase::getLabelEntryIfExists(int segmentation_id,
                                             LabelEntry* label_entry) const {
  auto it = labels_.find(segmentation_id);
  if (it != labels_.end()) {
    CHECK_NOTNULL(label_entry);
    *label_entry = *it->second;
    return true;
  }
  return false;
}

size_t LabelHandlerBase::numberOfLabels() const { return labels_.size(); }

}  // namespace panoptic_mapping
