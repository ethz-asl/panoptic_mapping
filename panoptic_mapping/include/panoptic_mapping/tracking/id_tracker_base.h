#ifndef PANOPTIC_MAPPING_TRACKING_ID_TRACKER_BASE_H_
#define PANOPTIC_MAPPING_TRACKING_ID_TRACKER_BASE_H_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/globals.h"
#include "panoptic_mapping/common/input_data_user.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/submap_allocation/freespace_allocator_base.h"
#include "panoptic_mapping/submap_allocation/submap_allocator_base.h"

namespace panoptic_mapping {

/**
 * @brief This class tracks and matches input to submap ids and allocates new
 * submaps where necessary using its submap allocators.
 */
class IDTrackerBase : public InputDataUser {
 public:
  explicit IDTrackerBase(std::shared_ptr<Globals> globals)
      : globals_(std::move(globals)) {}
  ~IDTrackerBase() override = default;

  // Interface.
  /**
   * @brief Changes the id image of the input data to track and match existing
   * submaps and allocate new submaps where necessary.
   *
   * @param submaps The submap collection to track against.
   * @param input The input data to process.
   */
  virtual void processInput(SubmapCollection* submaps, InputData* input) = 0;

  // Setters for external setup.
  /**
   * @brief Sets a callback that is called whenever an image needs to be
   * visualized.
   *
   * @param callback The function that is called to visualize images.
   */
  void setVisualizationCallback(
      std::function<void(const cv::Mat&, const std::string&)> callback) {
    visualization_callback_ = std::move(callback);
    visualize_ = true;
  }

  /**
   * @brief Set the submap allocator that is used to allocate non-freespace
   * maps.
   *
   * @param submap_allocator The submap allocator to be used.
   */
  void setSubmapAllocator(
      std::shared_ptr<SubmapAllocatorBase> submap_allocator) {
    submap_allocator_ = std::move(submap_allocator);
  }

  /**
   * @brief Set the submap allocator that is used to allocate freespace maps.
   *
   * @param submap_allocator The submap allocator to be used.
   */
  void setFreespaceAllocator(
      std::shared_ptr<FreespaceAllocatorBase> submap_allocator) {
    freespace_allocator_ = std::move(submap_allocator);
  }

 protected:
  // Members for access.
  std::shared_ptr<Globals> globals_;
  std::shared_ptr<SubmapAllocatorBase> submap_allocator_;
  std::shared_ptr<FreespaceAllocatorBase> freespace_allocator_;

  // Visualization
  bool visualizationIsOn() const { return visualize_; }
  void visualize(const cv::Mat& image, const std::string& name) {
    if (visualize_) {
      visualization_callback_(image, name);
    }
  }

 private:
  bool visualize_ = false;
  std::function<void(const cv::Mat&, const std::string&)>
      visualization_callback_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TRACKING_ID_TRACKER_BASE_H_
