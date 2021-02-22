#ifndef PANOPTIC_MAPPING_COMMON_INDEX_GETTER_H_
#define PANOPTIC_MAPPING_COMMON_INDEX_GETTER_H_

#include <mutex>
#include <utility>
#include <vector>

#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {

/**
 * Thread safe queue where threads can get the next index to process.
 */
template <typename IndexT>
class IndexGetter {
 public:
  explicit IndexGetter(std::vector<IndexT> indices)
      : indices_(std::move(indices)), current_index_(0) {}
  bool getNextIndex(IndexT* index) {
    CHECK_NOTNULL(index);
    mutex_.lock();
    if (current_index_ >= indices_.size()) {
      mutex_.unlock();
      return false;
    }
    *index = indices_[current_index_];
    current_index_++;
    mutex_.unlock();
    return true;
  }

 private:
  std::mutex mutex_;
  std::vector<IndexT> indices_;
  size_t current_index_;
};

typedef IndexGetter<int> SubmapIndexGetter;

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_INDEX_GETTER_H_
