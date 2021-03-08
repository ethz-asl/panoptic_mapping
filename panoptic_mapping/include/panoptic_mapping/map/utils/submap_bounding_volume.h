#ifndef PANOPTIC_MAPPING_MAP_UTILS_SUBMAP_BOUNDING_VOLUME_H_
#define PANOPTIC_MAPPING_MAP_UTILS_SUBMAP_BOUNDING_VOLUME_H_

#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {

class Submap;

/**
 * This class interfaces conservative bounding volumes to hierarchically
 * prune submaps. Implemented using an inexact conservative sphere
 * approximation. The bounding volume is owned by the submap it references.
 */
class SubmapBoundingVolume {
 public:
  explicit SubmapBoundingVolume(const Submap& submap);
  ~SubmapBoundingVolume() = default;

  // Interaction.
  void update();
  bool contains_S(const Point& point_S) const;
  bool contains_M(const Point& point_M) const;
  bool intersects(const SubmapBoundingVolume& other) const;
  bool isInsidePlane_S(const Point& normal_S) const;
  bool isInsidePlane_M(const Point& normal_M) const;

  // Access.
  FloatingPoint getRadius() const { return radius_; }
  const Point& getCenter() const { return center_; }

 private:
  const Submap* const submap_;
  Point center_;  // This is in submap frame.
  FloatingPoint radius_;
  size_t num_previous_blocks_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_UTILS_SUBMAP_BOUNDING_VOLUME_H_
