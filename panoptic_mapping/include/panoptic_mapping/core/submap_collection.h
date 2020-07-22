#ifndef PANOPTIC_MAPPING_CORE_SUBMAP_COLLECTION_H_
#define PANOPTIC_MAPPING_CORE_SUBMAP_COLLECTION_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/core/submap.h"

namespace panoptic_mapping {

/***
 * This class contains and manages access to all submaps.
 */
class SubmapCollection {
 public:
  SubmapCollection() = default;
  virtual ~SubmapCollection() = default;

  // iterator over submaps
  std::vector<std::unique_ptr<Submap>>::iterator begin() {
    return submaps_.begin();
  }
  std::vector<std::unique_ptr<Submap>>::iterator end() {
    return submaps_.end();
  }

  // modify the collection
  void addSubmap(std::unique_ptr<Submap> submap);
  Submap* createSubmap(double voxel_size, int voxels_per_side);
  bool removeSubmap(int id);
  bool submapIdExists(int id);    // check whether id exists
  Submap& getSubmap(int id);      // this assumes that the id exists
  void clear();

  // accessors
  size_t size() { return submaps_.size(); }

 private:
  std::vector<std::unique_ptr<Submap>> submaps_;
  std::unordered_map<int, size_t> id_to_index_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_SUBMAP_COLLECTION_H_
