#include "panoptic_mapping/map/submap_id.h"

namespace panoptic_mapping {

SubmapID::SubmapID(SubmapIDManager* manager)
    : manager_(manager), id_(manager->requestID()) {}

SubmapID::SubmapID(int id, SubmapIDManager* manager)
    : manager_(manager), id_(id) {}

SubmapID::~SubmapID() { manager_->releaseID(id_); }

SubmapIDManager::SubmapIDManager() : current_id_(0) {}

int SubmapIDManager::requestID() {
  if (vacant_ids_.empty()) {
    return current_id_++;
  } else {
    int id = vacant_ids_.front();
    vacant_ids_.pop();
    return id;
  }
}

void SubmapIDManager::releaseID(int id) {
  if (kAllowIDReuse) {
    vacant_ids_.push(id);
  }
}

}  // namespace panoptic_mapping
