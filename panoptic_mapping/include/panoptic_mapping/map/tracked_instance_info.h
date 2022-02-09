#ifndef PANOPTIC_MAPPING_MAP_TRACKED_INSTANCE_INFO_H_
#define PANOPTIC_MAPPING_MAP_TRACKED_INSTANCE_INFO_H_

#include <unordered_map>

#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {

struct SegmentInfo {
  int class_id;
  float instance_score;
  float class_score;
  std::vector<float> class_probs;
};

/**
 * @brief Tracked instance interface
 *
 */
struct TrackedInstanceInfo {
  enum class Type { kCount = 0, kUncertainty };

  /**
   * @brief
   *
   * @param matched_segment_info
   * @param matching_score
   */
  virtual void update(const SegmentInfo& matched_segment_info,
                      float matching_score) = 0;

  /**
   * @brief Get the Class ID object
   *
   * @return int
   */
  virtual int getClassID() const = 0;

  static std::shared_ptr<TrackedInstanceInfo> make_tracked_instance_info(
      Type type);
};

/**
 * @brief
 *
 */
struct CountTrackedInstanceInfo : public TrackedInstanceInfo {
  double score = 0.f;
  int current_index = 0;
  ClassificationCount total_count = 0;
  ClassificationCount current_count = 0;
  std::unordered_map<int, ClassificationCount> class_counts;

  void update(const SegmentInfo& matched_segment_info,
              float matching_score) override;

  int getClassID() const override { return current_index; }
};

struct UncertaintyTrackedInstanceInfo : public TrackedInstanceInfo {
  std::vector<float> class_probs;
  int current_index = 0;

  UncertaintyTrackedInstanceInfo() : class_probs(kNumClasses) {}

  void update(const SegmentInfo& matched_segment_info,
              float matching_score) override;

  int getClassID() const override { return current_index; }

  static size_t numClasses();
  static void setNumClasses(size_t num_classes);

 private:
  static size_t kNumClasses;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_TRACKED_INSTANCE_INFO_H_