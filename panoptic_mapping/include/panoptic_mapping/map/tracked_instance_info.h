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
  enum class Type : int { kCount = 0, kUncertainty = 1, kBayesian = 2 };

  /**
   * @brief
   *
   * @param matched_segment_info
   * @param matching_score
   */
  virtual void update(const SegmentInfo& matched_segment_info,
                      float matching_score) = 0;

  virtual Type getType() const = 0;

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
  int current_index = 0;
  ClassificationCount total_count = 0;
  ClassificationCount current_count = 0;
  std::unordered_map<int, ClassificationCount> class_counts;

  void update(const SegmentInfo& matched_segment_info,
              float matching_score) override;

  Type getType() const override { return Type::kCount; }

  int getClassID() const override { return current_index; }
};

/**
 * @brief Implements thing label integration in the PanopticFusion style
 *
 */
struct UncertaintyTrackedInstanceInfo : public TrackedInstanceInfo {
  UncertaintyTrackedInstanceInfo() {}

  void update(const SegmentInfo& matched_segment_info,
              float matching_score) override;

  Type getType() const override { return Type::kUncertainty; }

  int getClassID() const override { return current_index; }

 private:
  std::unordered_map<int, std::pair<double, double>> class_probs;
  int current_index = 0;
  int current_max_prob = 0.0f;
};

struct BayesianTrackedInstanceInfo : public TrackedInstanceInfo {
  BayesianTrackedInstanceInfo()
      : class_probs(kNumClasses + 1 /* Last entry is the void label */, 0.f) {}

  void update(const SegmentInfo& matched_segment_info,
              float matching_score) override;

  Type getType() const override { return Type::kBayesian; }

  int getClassID() const override {
    // The last entry represents void i.e. the ignore label
    return (current_index + 1) % (kNumClasses + 1);
  }

  static void setNumClasses(size_t num_classes);

 private:
  static size_t kNumClasses;

  int current_index = 0;
  float current_index_prob = 0.f;
  std::vector<float> class_probs;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_TRACKED_INSTANCE_INFO_H_