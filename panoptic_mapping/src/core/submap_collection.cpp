#include "panoptic_mapping/core/submap_collection.h"

#include <memory>
#include <utility>

namespace panoptic_mapping {

void SubmapCollection::addSubmap(std::unique_ptr<Submap> submap) {
  id_to_index_[submap->getID()] = submaps_.size();
  submaps_.emplace_back(std::move(submap));
}

Submap* SubmapCollection::createSubmap(const Submap::Config& config) {
  submaps_.emplace_back(std::make_unique<Submap>(config));
  Submap* new_submap = submaps_.back().get();
  id_to_index_[new_submap->getID()] = submaps_.size() - 1;
  return new_submap;
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

bool SubmapCollection::submapIdExists(int id) const {
  return id_to_index_.find(id) != id_to_index_.end();
}

Submap& SubmapCollection::getSubmap(int id) {
  // This assumes we checked that the id exists
  return *submaps_[id_to_index_[id]];
}

void SubmapCollection::clear() {
  submaps_.clear();
  id_to_index_.clear();
}

}  // namespace panoptic_mapping
