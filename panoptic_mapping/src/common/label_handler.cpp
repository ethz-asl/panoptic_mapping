#include "panoptic_mapping/common/label_handler.h"

#include <sstream>
#include <string>

#include "panoptic_mapping/3rd_party/csv.h"

namespace panoptic_mapping {

std::string LabelHandler::LabelEntry::toString() const {
  std::stringstream ss;
  ss << "InstanceID: " << segmentation_id << ", ClassID: " << class_id
     << ", PanopticID: " << panopticLabelToString(label)
     << ", Color: " << static_cast<int>(color.r) << " "
     << static_cast<int>(color.g) << " " << static_cast<int>(color.b)
     << ", Name: " << name << ", SuperCategory: " << supercategory;
  return ss.str();
}

void LabelHandler::readLabelsFromFile(const std::string& source_file) {
  // Currently assumes fixed header names in the target file. Reading
  // exceptions should be handled by the CSVReader.
  ir_to_instance_id_.clear();
  labels_.clear();

  // Read all possible columns and write the present ones.
  io::CSVReader<11> in(source_file);
  in.read_header(io::ignore_missing_column, "InstanceID", "ClassID",
                 "PanopticID", "MeshID", "R", "G", "B", "Name", "SuperCategory",
                 "InfraredID", "RIOGlobalID");

  bool read_row = true;
  while (read_row) {
    std::string name, supercat;
    int inst = -1, cls = -1, pan = -1, mesh = -1, r = -1, g = -1, b = -1,
        ir = -1, rio = -1;
    read_row =
        in.read_row(inst, cls, pan, mesh, r, g, b, name, supercat, ir, rio);
    // Label.
    LabelEntry label;
    if (inst != -1) {
      label.segmentation_id = inst;
    } else {
      continue;
    }
    if (cls != -1) {
      label.class_id = cls;
    }
    if (pan != -1) {
      label.label = pan ? PanopticLabel::kInstance : PanopticLabel::kBackground;
    }
    if (!name.empty()) {
      label.name = name;
    }
    if (!supercat.empty()) {
      label.supercategory = supercat;
    }
    if (r != -1 && g != -1 && b != -1) {
      label.color = voxblox::Color(r, g, b);
    }
    labels_[inst] = label;

    // Mesh.
    if (ir != -1) {
      ir_to_instance_id_[ir] = inst;
    }
  }
  LOG(INFO) << "Read " << labels_.size() << " labels from '" << source_file
            << "'.";

  // Unknown labels are referred to -1.
  labels_[-1] = LabelEntry();
}

bool LabelHandler::segmentationIdExists(int segmentation_id) const {
  return labels_.find(segmentation_id) != labels_.end();
}

int LabelHandler::getSegmentationIdFromIrId(int ir_id) const {
  auto it = ir_to_instance_id_.find(ir_id);
  if (it != ir_to_instance_id_.end()) {
    return it->second;
  } else {
    return -1;  // Label for unknown things.
  }
}

int LabelHandler::getClassID(int segmentation_id) const {
  return labels_.at(segmentation_id).class_id;
}

bool LabelHandler::isBackgroundClass(int segmentation_id) const {
  return labels_.at(segmentation_id).label == PanopticLabel::kBackground;
}

bool LabelHandler::isInstanceClass(int segmentation_id) const {
  return labels_.at(segmentation_id).label == PanopticLabel::kInstance;
}

bool LabelHandler::isUnknownClass(int segmentation_id) const {
  return labels_.at(segmentation_id).label == PanopticLabel::kUnknown;
}

bool LabelHandler::isSpaceClass(int segmentation_id) const {
  return labels_.at(segmentation_id).label == PanopticLabel::kFreeSpace;
}

PanopticLabel LabelHandler::getPanopticLabel(int segmentation_id) const {
  return labels_.at(segmentation_id).label;
}

const voxblox::Color& LabelHandler::getColor(int segmentation_id) const {
  return labels_.at(segmentation_id).color;
}

const std::string& LabelHandler::getName(int segmentation_id) const {
  return labels_.at(segmentation_id).name;
}

const std::string& LabelHandler::getSuperCategory(int segmentation_id) const {
  return labels_.at(segmentation_id).supercategory;
}

const LabelHandler::LabelEntry& LabelHandler::getLabelEntry(
    int segmentation_id) const {
  return labels_.at(segmentation_id);
}

}  // namespace panoptic_mapping
