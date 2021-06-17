#ifndef PANOPTIC_MAPPING_MAP_INSTANCE_ID_H_
#define PANOPTIC_MAPPING_MAP_INSTANCE_ID_H_

#include <unordered_map>

namespace panoptic_mapping {

/**
 * Class for ID management. Default uses a global singleton but e.g. individual
 * submap collections are allowed to have separate.
 */
class InstanceIDManager {
 public:
  InstanceIDManager();

  // Default is a global instance id manager, as singleton.
  static InstanceIDManager* getGlobalInstance() {
    static InstanceIDManager instance;
    return &instance;
  }

 private:
  friend class InstanceID;

  // Interaction
  int requestID();
  void registerID(int id);
  void releaseID(int id);

 private:
  static const bool kAllowIDReuse = false;

  // Tracking.
  int current_id_;
  std::unordered_map<int, int> used_ids_;

  void increment(int id);
  bool decrement(int id);
};

/**
 * This class instantiates and tracks instance IDs. Instance IDs are allowed to
 * be duplicates but by default generate new unique ids.
 */
class InstanceID {
 public:
  // controlled con- and destruction.
  explicit InstanceID(
      InstanceIDManager* manager = InstanceIDManager::getGlobalInstance());
  explicit InstanceID(int id, InstanceIDManager* manager =
                                  InstanceIDManager::getGlobalInstance());
  InstanceID(const InstanceID& other);
  ~InstanceID();

  // Conversion and assignment.
  operator int() const { return id_; }
  InstanceID& operator=(const int& id);

 private:
  int id_;
  InstanceIDManager* const manager_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_INSTANCE_ID_H_
