#ifndef PANOPTIC_MAPPING_PREPROCESSING_LABEL_HANDLER_H_
#define PANOPTIC_MAPPING_PREPROCESSING_LABEL_HANDLER_H_

#include <map>
#include <string>

#include <voxblox/core/color.h>

#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/common.h"

namespace panoptic_mapping {

class LabelHandler {
 public:
  struct Label{
    int segmentation_id = 0;
    int class_label = 0;
    bool is_background_class = true;
    std::string name = "Name is not initialized";
    voxblox::Color color;
  };

  LabelHandler() = default;

  void readLabelsFromFile(const std::string& source_file);

  // these return true if the id was found
  bool segmentationIdExists(int segmentation_id) const;

  // these acessors assume that the segmentation_id exists
  int getClassLabel(int segmentation_id) const;
  bool isBackgroundClass(int segmentation_id) const;
  bool isInstanceClass(int segmentation_id) const;
  const voxblox::Color& getColor(int segmentation_id) const;
  const std::string& getName(int segmentation_id) const;
  const Label& getLabel(int segmentation_id) const;

 private:
  // list of the labels associated with each segmentation id
  std::unordered_map<int, Label> labels_;
};

} // namespace panoptic_mapping

#endif //PANOPTIC_MAPPING_PREPROCESSING_LABEL_HANDLER_H_
