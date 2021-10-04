#include "panoptic_mapping/map/submap_collection.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "panoptic_mapping/SubmapCollection.pb.h"

namespace panoptic_mapping {

Submap* SubmapCollection::createSubmap(const Submap::Config& config) {
  submaps_.emplace_back(std::make_unique<Submap>(config, &submap_id_manager_,
                                                 &instance_id_manager_));
  Submap* new_submap = submaps_.back().get();
  id_to_index_[new_submap->getID()] = submaps_.size() - 1;
  return new_submap;
}

bool SubmapCollection::removeSubmap(int id) {
  auto it = id_to_index_.find(id);
  if (it == id_to_index_.end()) {
    // Submap does not exist.
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

Submap* SubmapCollection::getSubmapPtr(int id) {
  // This assumes we checked that the id exists.
  return submaps_[id_to_index_.at(id)].get();
}

void SubmapCollection::clear() {
  submaps_.clear();
  id_to_index_.clear();
  instance_id_manager_ = InstanceIDManager();
  submap_id_manager_ = SubmapIDManager();
  instance_to_submap_ids_.clear();
  active_freespace_submap_id_ = -1;
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

void SubmapCollection::updateInstanceToSubmapIDTable() {
  // Clear the table and add all new elements.
  instance_to_submap_ids_ = std::unordered_map<int, std::unordered_set<int>>();
  for (const Submap& submap : *this) {
    instance_to_submap_ids_[submap.getInstanceID()].emplace(submap.getID());
  }
}

// Save load functionality was heavily adapted from cblox.
bool SubmapCollection::saveToFile(const std::string& file_path) const {
  CHECK(!file_path.empty());
  std::fstream outfile;
  outfile.open(file_path, std::fstream::out | std::fstream::binary);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file '" << file_path
               << "' to save the submap collection.";
    return false;
  }

  // Saving the submap collection header object.
  SubmapCollectionProto submap_collection_proto;
  submap_collection_proto.set_num_submaps(submaps_.size());
  if (!voxblox::utils::writeProtoMsgToStream(submap_collection_proto,
                                             &outfile)) {
    LOG(ERROR) << "Could not write submap collection header message.";
    outfile.close();
    return false;
  }

  // Saving the submaps.
  for (const auto& submap : submaps_) {
    if (!submap->saveToStream(&outfile)) {
      LOG(WARNING) << "Failed to save submap with ID '" << submap->getID()
                   << "'.";
      outfile.close();
      return false;
    }
  }
  outfile.close();
  return true;
}

bool SubmapCollection::loadFromFile(const std::string& file_path,
                                    bool recompute_data) {
  CHECK(!file_path.empty());
  // Clear the current maps.
  submaps_.clear();

  // Open and check the file.
  std::ifstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file '" << file_path << "'.";
    return false;
  }

  // Unused byte offset result.
  uint64_t tmp_byte_offset = 0u;
  SubmapCollectionProto submap_collection_proto;
  if (!voxblox::utils::readProtoMsgFromStream(
          &proto_file, &submap_collection_proto, &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read the protobuf message.";
    return false;
  }

  // Loading each of the submaps.
  for (size_t sub_map_index = 0u;
       sub_map_index < submap_collection_proto.num_submaps(); ++sub_map_index) {
    std::unique_ptr<Submap> submap_ptr =
        Submap::loadFromStream(&proto_file, &tmp_byte_offset);
    if (submap_ptr == nullptr) {
      LOG(ERROR) << "Failed to load submap '" << sub_map_index
                 << "' from stream.";
      proto_file.close();
      return false;
    }

    // Add to the collection.
    id_to_index_[submap_ptr->getID()] = submaps_.size();
    submaps_.emplace_back(std::move(submap_ptr));
  }
  proto_file.close();

  // Recompute data that is not stored with the submap.
  if (recompute_data) {
    for (Submap& submap : *this) {
      submap.updateEverything(false);
    }
  }
  return true;
}

std::unique_ptr<SubmapCollection> SubmapCollection::clone() const {
  std::unique_ptr<SubmapCollection> result =
      std::make_unique<SubmapCollection>();

  // Copy all the meta data.
  result->submap_id_manager_ = submap_id_manager_;
  result->instance_id_manager_ = instance_id_manager_;
  result->id_to_index_ = id_to_index_;
  result->instance_to_submap_ids_ = instance_to_submap_ids_;
  result->active_freespace_submap_id_ = active_freespace_submap_id_;

  // Deep copy all the submaps to the new managers.
  for (const Submap& submap : *this) {
    result->submaps_.emplace_back(submap.clone(&result->submap_id_manager_,
                                               &result->instance_id_manager_));
  }

  return result;
}

}  // namespace panoptic_mapping
