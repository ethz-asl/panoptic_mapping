#include "panoptic_mapping/core/submap_collection.h"

namespace panoptic_mapping {

bool SubmapCollection::addSubmap(const Submap &submap) {
  if (submapIdExists(submap.getID())) {
    LOG(ERROR) << "Can not add submap, ID '" << submap.getID() << "' already exists.";
    return false;
  }
  id_to_index_[submap.getID()] = submaps_.size();
  submaps_.push_back(submap);
  return true;
}

bool SubmapCollection::removeSubmap(int id) {
  auto it = id_to_index_.find(id);
  if (it == id_to_index_.end()) {
    //submap does not exist
    return false;
  }
  submaps_.erase(submaps_.begin() + it->second);
  return true;
}

bool SubmapCollection::changeSubmapID(int id_old, int id_new) {
  if (submapIdExists(id_new)) { return false; }
  if (!submapIdExists(id_old)) { return false; }
  submaps_[id_to_index_[id_old]].id_ = id_new;
  return true;
}

bool SubmapCollection::submapIdExists(int id) {
  return id_to_index_.find(id) != id_to_index_.end();
}

Submap &SubmapCollection::getSubmap(int id) {
  // This assumes we checked that the id exists
  return submaps_[id_to_index_[id]];
}

void SubmapCollection::clear(){
  submaps_.clear();
  id_to_index_.clear();
}

} // namespace panoptic_mapping
