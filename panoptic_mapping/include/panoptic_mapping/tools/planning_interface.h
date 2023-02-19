#ifndef PANOPTIC_MAPPING_TOOLS_PLANNING_INTERFACE_H_
#define PANOPTIC_MAPPING_TOOLS_PLANNING_INTERFACE_H_

#include <memory>

#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * @brief This class implements a high level interfaces for lookups on the
 * submap collection.
 */
class PlanningInterface {
 public:
  explicit PlanningInterface(std::shared_ptr<const SubmapCollection> submaps);

  enum class VoxelState {
    kUnknown = 0,
    kKnownFree,
    kKnownOccupied,
    kPersistentOccupied,
    kExpectedFree,
    kExpectedOccupied,
  };

  // Access.
  const SubmapCollection& getSubmapCollection() const { return *submaps_; }

  // Lookups.
  /**
   * @brief Check if a point is observed in the map.
   *
   * @param position Position of point in world frame.
   * @param consider_change_state If false considers only submaps considered
   * present, i.e. whose ChangeState is active or persistent.
   * @param include_inactive_maps If false considers only active submaps.
   * @return True if the point was observed.
   */
  bool isObserved(const Point& position, bool consider_change_state = true,
                  bool include_inactive_maps = true) const;

  /**
   * @brief Compute the voxel state of a point in the map for planning.
   *
   * @param position Position of point in world frame.
   * @return VoxelState of the given point.
   */
  VoxelState getVoxelState(const Point& position) const;

  /**
   * @brief Computes the truncated signed distance function (TSDF) at a point in
   * the multi-resolution map.
   *
   * @param position Position of point in world frame.
   * @param distance Pointer to value to store the distance in.
   * @param consider_change_state If true considers only submaps considered
   * present, i.e. whose ChangeState is active or persistent.
   * @param include_free_space If true points lying only in free space are
   * considered observed.
   * @return True if the point was observed and thus a distance returned.
   */
  bool getDistance(const Point& position, float* distance,
                   bool consider_change_state = true,
                   bool include_free_space = true) const;

 private:
  std::shared_ptr<const SubmapCollection> submaps_;
  static constexpr float kObservedMinWeight_ = 1e-6;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_PLANNING_INTERFACE_H_
