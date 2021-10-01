#ifndef PANOPTIC_MAPPING_LABELS_LABEL_HANDLER_BASE_H_
#define PANOPTIC_MAPPING_LABELS_LABEL_HANDLER_BASE_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/labels/label_entry.h"

namespace panoptic_mapping {

/**
 * @brief Class that enables look-ups on labels via the segmentation ID. Exposes
 * the minimum required fields explicitly, however returning the LabelEntry
 * directly is more efficient for multiple lookups.
 */
class LabelHandlerBase {
 public:
  LabelHandlerBase() = default;
  virtual ~LabelHandlerBase() = default;

  // This returns true if the id was found.
  bool segmentationIdExists(int segmentation_id) const;

  // These acessors assume that the segmentation_id exists.
  int getClassID(int segmentation_id) const;
  bool isBackgroundClass(int segmentation_id) const;
  bool isInstanceClass(int segmentation_id) const;
  bool isUnknownClass(int segmentation_id) const;
  bool isSpaceClass(int segmentation_id) const;
  PanopticLabel getPanopticLabel(int segmentation_id) const;
  const voxblox::Color& getColor(int segmentation_id) const;
  const std::string& getName(int segmentation_id) const;
  const LabelEntry& getLabelEntry(int segmentation_id) const;

  /**
   * @brief Get the LabelEntry if it exists in a combined lookup.
   *
   * @param segmentation_id Segmentation ID to look up.
   * @param label_entry Resulting LabelEntry, set if the look-up was successful.
   * @return True if the segmentation ID was found. False otherwise.
   */
  bool getLabelEntryIfExists(int segmentation_id,
                             LabelEntry* label_entry) const;

  // Get the number of stored labels.
  size_t numberOfLabels() const;

 protected:
  // List of the labels associated with each segmentation ID. Labels are stored
  // by pointer such that derived label types can also be stored here.
  std::unordered_map<int, std::unique_ptr<LabelEntry>> labels_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_LABELS_LABEL_HANDLER_BASE_H_
