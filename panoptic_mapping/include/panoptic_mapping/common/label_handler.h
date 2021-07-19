#ifndef PANOPTIC_MAPPING_COMMON_LABEL_HANDLER_H_
#define PANOPTIC_MAPPING_COMMON_LABEL_HANDLER_H_

#include <map>
#include <string>
#include <unordered_map>

#include <voxblox/core/color.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"

namespace panoptic_mapping {

class LabelHandler {
 public:
  struct LabelEntry {
    // The default lable is unknown.
    int segmentation_id = -1;
    int class_id = -1;
    PanopticLabel label = PanopticLabel::kUnknown;
    std::string name = "UninitializedName";
    std::string supercategory = "UninitializedSupercategory";
    std::string size = "Unknown";
    Color color = Color(80, 80, 80);

    // Print the contents of a label.
    std::string toString() const;
  };

  LabelHandler() = default;

  // Setup.
  void readLabelsFromFile(const std::string& source_file);

  // This returns true if the id was found.
  bool segmentationIdExists(int segmentation_id) const;
  int getSegmentationIdFromMeshId(int mesh_id) const;

  // These acessors assume that the segmentation_id exists.
  int getClassID(int segmentation_id) const;
  bool isBackgroundClass(int segmentation_id) const;
  bool isInstanceClass(int segmentation_id) const;
  bool isUnknownClass(int segmentation_id) const;
  bool isSpaceClass(int segmentation_id) const;
  PanopticLabel getPanopticLabel(int segmentation_id) const;
  const voxblox::Color& getColor(int segmentation_id) const;
  const std::string& getName(int segmentation_id) const;
  const std::string& getSuperCategory(int segmentation_id) const;
  const LabelEntry& getLabelEntry(int segmentation_id) const;

 private:
  // List of the labels associated with each segmentation id.
  std::unordered_map<int, LabelEntry> labels_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_LABEL_HANDLER_H_
