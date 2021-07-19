#ifndef PANOPTIC_MAPPING_TOOLS_THREAD_SAFE_SUBMAP_COLLECTION_H_
#define PANOPTIC_MAPPING_TOOLS_THREAD_SAFE_SUBMAP_COLLECTION_H_

#include <memory>
#include <utility>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * This class wraps a submap collection for thread-safe read-only access by
 * simply copying the original submap collection.
 */

class ThreadSafeSubmapCollection {
 public:
  /* Construction */
  explicit ThreadSafeSubmapCollection(std::shared_ptr<SubmapCollection> submaps)
      : submaps_source_(std::move(submaps)), updated_(new bool()) {
    update();
  }
  virtual ~ThreadSafeSubmapCollection() = default;

  /* Interaction */
  // Update the lookup submaps based on the source submaps.
  void update() {
    Timer timer("tools/thread_safe_submap_collection/update");
    submaps_ = submaps_source_->clone();
    timer.Stop();
    *updated_ = true;
  }

  // Check whether there were any updates since the last lookup.
  bool wasUpdated() const { return *updated_; }

  // Get thread-safe read-only access to the collection
  const SubmapCollection& getSubmaps() const {
    *updated_ = false;
    return *submaps_;
  }

 private:
  // Reference to the original submap collection.
  std::shared_ptr<SubmapCollection> submaps_source_;

  // Deep copy of the submap collection for thread-safe read-only access.
  std::unique_ptr<SubmapCollection> submaps_;

  // Track whether the submaps were updated since last lookup.
  std::unique_ptr<bool> updated_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_THREAD_SAFE_SUBMAP_COLLECTION_H_
