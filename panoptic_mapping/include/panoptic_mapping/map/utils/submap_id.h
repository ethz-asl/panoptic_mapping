#ifndef PANOPTIC_MAPPING_MAP_UTILS_SUBMAP_ID_H_
#define PANOPTIC_MAPPING_MAP_UTILS_SUBMAP_ID_H_

#include <queue>

namespace panoptic_mapping {

/**
 * This class instantiates and tracks unique submap IDs.
 */
class SubmapID {
 public:
  // controlled con- and destruction
  SubmapID();
  ~SubmapID();

  SubmapID(const SubmapID&) = delete;
  SubmapID& operator=(const SubmapID&) = delete;

  operator int() const { return id_; }

 private:
  const int id_;
};

/**
 * Singleton for global ID management.
 */
class SubmapIDManager {
 public:
  // accessor
  static SubmapIDManager& getInstance() {
    static SubmapIDManager instance;
    return instance;
  }

  // prevent copies
  SubmapIDManager(SubmapIDManager const&) = delete;
  void operator=(SubmapIDManager const&) = delete;

  // interaction
  int requestID();
  void releaseID(int id);

 private:
  SubmapIDManager();

  // Currently submap IDs are assumed to be unique over the lifetime of a run.
  static const bool kAllowIDReuse = false;

  int current_id_;
  std::queue<int> vacant_ids_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_UTILS_SUBMAP_ID_H_
