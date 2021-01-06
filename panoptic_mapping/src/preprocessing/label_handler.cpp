#include "panoptic_mapping/preprocessing/label_handler.h"

#include <fstream>
#include <string>

#include "panoptic_mapping/3rd_party/csv.h"

namespace panoptic_mapping {

void LabelHandler::readLabelsFromFile(const std::string& source_file) {
  // Currently assumes fixed header names in the target file. Reading
  // exceptions should be handled by the CSVReader.
  mesh_to_instance_id_.clear();
  labels_.clear();

  // Read data.
  io::CSVReader<8> in(source_file);
  in.read_header(io::ignore_extra_column, "InstanceID", "ClassID", "PanopticID",
                 "MeshID", "R", "G", "B", "Name");
  std::string name;
  int inst, cls, pan, mesh, r, g, b;
  while (in.read_row(inst, cls, pan, mesh, r, g, b, name)) {
    // Label.
    LabelEntry label;
    label.segmentation_id = inst;
    label.class_id = cls;
    label.label = pan ? PanopticLabel::kInstance : PanopticLabel::kBackground;
    label.name = name;
    label.color = voxblox::Color(r, g, b);
    labels_[inst] = label;

    // Mesh.
    mesh_to_instance_id_[mesh] = inst;
  }
  LOG(INFO) << "Read " << labels_.size() << " labels from '" << source_file
            << "'.";

  // 255 is reserved for unknown labels.
  LabelEntry label;
  label.segmentation_id = 255;
  label.class_id = 0;
  label.label = PanopticLabel::kUnknown;
  label.name = "Unknown";
  label.color = voxblox::Color(80, 80, 80);
  labels_[255] = label;
}

bool LabelHandler::segmentationIdExists(int segmentation_id) const {
  return labels_.find(segmentation_id) != labels_.end();
}

int LabelHandler::getSegmentationIdFromMeshId(int mesh_id) const {
  auto it = mesh_to_instance_id_.find(mesh_id);
  if (it != mesh_to_instance_id_.end()) {
    return it->second;
  } else {
    return 255;  // Label reserved for unknown things.
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

bool LabelHandler::isUknownClass(int segmentation_id) const {
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

const LabelHandler::LabelEntry& LabelHandler::getLabelEntry(
    int segmentation_id) const {
  return labels_.at(segmentation_id);
}

}  // namespace panoptic_mapping
