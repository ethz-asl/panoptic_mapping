#include "panoptic_mapping/core/submap_collection.h"

#include <memory>
#include <utility>
#include <vector> for vector<>

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

const Submap& SubmapCollection::getSubmap(int id) const {
  // This assumes we checked that the id exists.
  return *submaps_[id_to_index_.at(id)];
}

Submap* SubmapCollection::getSubmapPtr(int id) const {
  // This assumes we checked that the id exists.
  return submaps_[id_to_index_.at(id)].get();
}

void SubmapCollection::clear() {
  submaps_.clear();
  id_to_index_.clear();
}

void SubmapCollection::updateIDList(const std::vector<int>& id_list,
                                    std::vector<int>* new_ids,
                                    std::vector<int>* deleted_ids) const {
  CHECK_NOTNULL(new_ids);
  CHECK_NOTNULL(deleted_ids);
  // Find all deleted submaps.
  for (const int& id : id_list) {
    if (!submapIdExists(id)) {
      deleted_ids->emplace_back(id);
    }
  }
  // Find all new submaps.
  for (const auto& id_submap_pair : id_to_index_) {
    auto it = std::find(id_list.begin(), id_list.end(), id_submap_pair.first);
    if (it == id_list.end()) {
      new_ids->emplace_back(id_submap_pair.first);
    }
  }
}

}  // namespace panoptic_mapping
