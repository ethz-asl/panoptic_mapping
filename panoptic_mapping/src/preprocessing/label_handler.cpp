#include "panoptic_mapping/preprocessing/label_handler.h"

#include <fstream>
#include <string>

#include "panoptic_mapping/3rd_party/csv.h"

namespace panoptic_mapping {

void LabelHandler::readLabelsFromFile(const std::string& source_file) {
  // currently assumes a fixed header names in the target file. Reading
  // exceptions should be handled by the CSVReader
  io::CSVReader<7> in(source_file);
  in.read_header(io::ignore_extra_column, "InstanceID", "ClassID", "PanopticID",
                 "R", "G", "B", "Name");
  std::string name;
  int inst, cls, pan, r, g, b;
  while (in.read_row(inst, cls, pan, r, g, b, name)) {
    Label label;
    label.segmentation_id = inst;
    label.class_label = cls;
    label.is_background_class = pan;
    label.name = name;
    label.color = voxblox::Color(r, g, b);
    labels_[inst] = label;
  }
  VLOG(1) << "Read " << labels_.size() << " labels from '" << source_file
          << "'.";
}

bool LabelHandler::segmentationIdExists(int segmentation_id) const {
  return labels_.find(segmentation_id) != labels_.end();
}

int LabelHandler::getClassLabel(int segmentation_id) const {
  return labels_.at(segmentation_id).class_label;
}

bool LabelHandler::isBackgroundClass(int segmentation_id) const {
  return labels_.at(segmentation_id).is_background_class;
}

bool LabelHandler::isInstanceClass(int segmentation_id) const {
  return !labels_.at(segmentation_id).is_background_class;
}

const voxblox::Color& LabelHandler::getColor(int segmentation_id) const {
  return labels_.at(segmentation_id).color;
}

const std::string& LabelHandler::getName(int segmentation_id) const {
  return labels_.at(segmentation_id).name;
}

const LabelHandler::Label& LabelHandler::getLabel(int segmentation_id) const {
  return labels_.at(segmentation_id);
}

}  // namespace panoptic_mapping
