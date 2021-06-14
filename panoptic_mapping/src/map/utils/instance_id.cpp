#include "panoptic_mapping/map/utils/instance_id.h"

namespace panoptic_mapping {

// By default create a new unique id.
InstanceID::InstanceID() : id_(InstanceIDManager::getInstance().requestID()) {}

InstanceID::~InstanceID() { InstanceIDManager::getInstance().releaseID(id_); }

InstanceID::InstanceID(int id) : id_(id) {
  InstanceIDManager::getInstance().registerID(id);
}

InstanceID::InstanceID(const InstanceID& other) {
  InstanceIDManager::getInstance().releaseID(id_);
  id_ = other.id_;
  InstanceIDManager::getInstance().registerID(id_);
}

InstanceID& InstanceID::operator=(const int& id) {
  InstanceIDManager::getInstance().releaseID(id_);
  id_ = id;
  InstanceIDManager::getInstance().registerID(id_);
  return *this;
}

InstanceIDManager::InstanceIDManager() : current_id_(0) {}

void InstanceIDManager::increment(int id) {
  auto it = used_ids_.find(id);
  if (it == used_ids_.end()) {
    used_ids_[id] = 1;
  } else {
    it->second++;
  }
}

bool InstanceIDManager::decrement(int id) {
  // Return true if the id was deleted (decrement to 0)
  auto it = used_ids_.find(id);
  if (it == used_ids_.end()) {
    return true;
  }
  it->second--;
  if (it->second <= 0) {
    used_ids_.erase(it);
    return true;
  }
  return false;
}

int InstanceIDManager::requestID() {
  // Find the next higher unique ID.
  while (used_ids_.find(current_id_) != used_ids_.end()) {
    current_id_++;
  }
  increment(current_id_);
  return current_id_;
}

void InstanceIDManager::registerID(int id) { increment(id); }

void InstanceIDManager::releaseID(int id) {
  const bool id_is_free = decrement(id);
  if (id_is_free && id < current_id_ && kAllowIDReuse) {
    current_id_ = id;
  }
}

}  // namespace panoptic_mapping
