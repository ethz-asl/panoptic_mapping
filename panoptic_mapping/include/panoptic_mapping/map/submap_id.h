#ifndef PANOPTIC_MAPPING_MAP_SUBMAP_ID_H_
#define PANOPTIC_MAPPING_MAP_SUBMAP_ID_H_

#include <queue>

namespace panoptic_mapping {

/**
 * @brief Class for ID management. Default uses a global singleton but e.g.
 * individual submap collections are allowed to have separate.
 */
class SubmapIDManager {
 public:
  SubmapIDManager();

  // Access to the global id manager via singleton.
  static SubmapIDManager* getGlobalInstance() {
    static SubmapIDManager instance;
    return &instance;
  }

 private:
  friend class SubmapID;

  // Interaction.
  int requestID();
  void releaseID(int id);

 private:
  // Currently submap IDs are assumed to be unique over the lifetime of a run.
  static const bool kAllowIDReuse = false;

  int current_id_;
  std::queue<int> vacant_ids_;
};

/**
 * @brief This class instantiates and tracks unique submap IDs.
 */
class SubmapID {
 public:
  // Controlled con- and destruction.
  explicit SubmapID(
      SubmapIDManager* manager = SubmapIDManager::getGlobalInstance());
  ~SubmapID();

  SubmapID(const SubmapID&) = delete;
  SubmapID& operator=(const SubmapID&) = delete;

  operator int() const { return id_; }

 private:
  friend class Submap;
  const int id_;
  SubmapIDManager* const manager_;

  // This constructor is specifically for copying submap ids from one id manager
  // to the other and does not double check uniqueness of the ids.
  SubmapID(int id, SubmapIDManager* manager);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_SUBMAP_ID_H_
