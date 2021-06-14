#include "panoptic_mapping/map/utils/submap_id.h"

namespace panoptic_mapping {

SubmapID::SubmapID() : id_(SubmapIDManager::getInstance().requestID()) {}

SubmapID::~SubmapID() { SubmapIDManager::getInstance().releaseID(id_); }

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
