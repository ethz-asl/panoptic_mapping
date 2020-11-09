#ifndef PANOPTIC_MAPPING_CORE_SUBMAP_BOUNDING_VOLUME_H_
#define PANOPTIC_MAPPING_CORE_SUBMAP_BOUNDING_VOLUME_H_

#include "panoptic_mapping/core/common.h"

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
  bool contains(const Point& point_S) const;
  bool intersects(const SubmapBoundingVolume& other) const;
  bool isInsidePlane(const Point& normal_S) const;

  // Access.
  FloatingPoint getRadius() const { return radius_; }
  const Point& getCenter() const { return center_; }

 private:
  const Submap* submap_;
  Point center_;  // This is in submap frame.
  FloatingPoint radius_ = 0.f;
  size_t num_previous_blocks_ = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_SUBMAP_BOUNDING_VOLUME_H_
