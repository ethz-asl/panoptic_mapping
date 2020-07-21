#include "panoptic_mapping/core/submap_collection.h"

namespace panoptic_mapping {

bool SubmapCollection::addSubmap(const Submap& submap) {
  if (submapIdExists(submap.getID())) {
    LOG(ERROR) << "Can not add submap, ID '" << submap.getID()
               << "' already exists.";
    return false;
  }
  id_to_index_[submap.getID()] = submaps_.size();
  submaps_.push_back(submap);
  return true;
}

bool SubmapCollection::removeSubmap(int id) {
  auto it = id_to_index_.find(id);
  if (it == id_to_index_.end()) {
    // submap does not exist
    return false;
  }
  size_t previous_index = it->second;
  submaps_.erase(submaps_.begin() + it->second);
  id_to_index_.erase(it);
  // correct the index table
  for (auto& id_index_pair : id_to_index_) {
    if (id_index_pair.second > previous_index) {
      id_index_pair.second -= 1;
    }
  }
  return true;
}

bool SubmapCollection::submapIdExists(int id) {
  return id_to_index_.find(id) != id_to_index_.end();
}

Submap& SubmapCollection::getSubmap(int id) {
  // This assumes we checked that the id exists
  return submaps_[id_to_index_[id]];
}

void SubmapCollection::clear() {
  submaps_.clear();
  id_to_index_.clear();
}

}  // namespace panoptic_mapping
