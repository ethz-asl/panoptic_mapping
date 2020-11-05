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

  // Iterators over submaps.
  std::vector<std::unique_ptr<Submap>>::const_iterator begin() const {
    return submaps_.begin();
  }
  std::vector<std::unique_ptr<Submap>>::const_iterator end() const {
    return submaps_.end();
  }

  // Modify the collection.
  void addSubmap(std::unique_ptr<Submap> submap);
  Submap* createSubmap(const Submap::Config& config);
  bool removeSubmap(int id);
  const Submap& getSubmap(int id) const;  // This assumes that the id exists.
  Submap* getSubmapPtr(int id) const;     // This assumes that the id exists.
  void clear();

  // Accessors.
  size_t size() const { return submaps_.size(); }
  bool submapIdExists(int id) const;  // Check whether id exists.

  // Tools.
  void updateIDList(const std::vector<int>& id_list, std::vector<int>* new_ids,
                    std::vector<int>* deleted_ids) const;

 private:
  std::vector<std::unique_ptr<Submap>> submaps_;
  std::unordered_map<int, size_t> id_to_index_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_SUBMAP_COLLECTION_H_
