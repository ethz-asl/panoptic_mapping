#ifndef PANOPTIC_MAPPING_PREPROCESSING_TRACKING_INFO_H_
#define PANOPTIC_MAPPING_PREPROCESSING_TRACKING_INFO_H_

#include <memory>
#include <unordered_map>
#include <vector>

namespace panoptic_mapping {

/**
 * Tool to aid tracking metrics computation / comparison.
 */

class TrackingInfo {
 public:
  explicit TrackingInfo(int input_id);
  ~TrackingInfo() = default;

  // Accessors.

  // Input.
  void insertRenderedID(int rendered_id);

 private:
  const int input_id_;

  // Raw data.
  int count = 0;
  std::unordered_map<int, int> rendered_count;
};

// Container for tracking infos.
class TrackingInfos {
 public:
  TrackingInfos() = default;
  ~TrackingInfos() = default;

  // Access.

  // Input.
  void insertIdPair(int input_id, int rendered_id);

 private:
  std::unordered_map<int, TrackingInfo> infos_;
  std::unordered_map<int, int> total_rendered_count_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_TRACKING_INFO_H_
