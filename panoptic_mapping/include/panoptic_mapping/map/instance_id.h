#ifndef PANOPTIC_MAPPING_MAP_INSTANCE_ID_H_
#define PANOPTIC_MAPPING_MAP_INSTANCE_ID_H_

#include <unordered_map>

namespace panoptic_mapping {

/**
 * This class instantiates and tracks instance IDs. Instance IDs are allowed to
 * be duplicates but by default generate new unique ids.
 */
class InstanceID {
 public:
  // controlled con- and destruction.
  InstanceID();
  explicit InstanceID(int id);
  InstanceID(const InstanceID& other);
  ~InstanceID();

  // Conversion and assignment.
  operator int() const { return id_; }
  InstanceID& operator=(const int& id);
  // NOTE(schmluk): Other assignment or move operators should call the copy
  // constructor, so no need to implement them.

 private:
  int id_;
};

/**
 * Singleton for global ID management.
 */
class InstanceIDManager {
 public:
  // accessor
  static InstanceIDManager& getInstance() {
    static InstanceIDManager instance;
    return instance;
  }

  // prevent copies
  InstanceIDManager(InstanceIDManager const&) = delete;
  void operator=(InstanceIDManager const&) = delete;

  // interaction
  int requestID();
  void registerID(int id);
  void releaseID(int id);

 private:
  InstanceIDManager();

  static const bool kAllowIDReuse = true;

  // Tracking.
  int current_id_;
  std::unordered_map<int, int> used_ids_;

  void increment(int id);
  bool decrement(int id);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_INSTANCE_ID_H_
