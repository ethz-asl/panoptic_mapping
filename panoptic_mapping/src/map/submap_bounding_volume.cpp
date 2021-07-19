#include "panoptic_mapping/map/submap_bounding_volume.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "panoptic_mapping/map/submap.h"

namespace panoptic_mapping {

SubmapBoundingVolume::SubmapBoundingVolume(const Submap& submap)
    : submap_(&submap),
      center_(0.f, 0.f, 0.f),
      radius_(0.f),
      num_previous_blocks_(0) {}

void SubmapBoundingVolume::update() {
  // A conservative approximation that computes the centroid from the
  // grid-aligned bounding box and then shrinks a sphere on it. This is
  // generally a significant over-estimation of the sphere, could use an exact
  // algorithm here if required (e.g. https://github.com/hbf/miniball or
  // https://people.inf.ethz.ch/gaertner/subdir/software/miniball.html).

  // Prevent redundant updates.
  if (submap_->getTsdfLayer().getNumberOfAllocatedBlocks() ==
      num_previous_blocks_) {
    return;
  } else {
    num_previous_blocks_ = submap_->getTsdfLayer().getNumberOfAllocatedBlocks();
  }

  // Setup.
  voxblox::BlockIndexList block_indices;
  submap_->getTsdfLayer().getAllAllocatedBlocks(&block_indices);
  if (block_indices.empty()) {
    radius_ = 0.f;
    center_ = Point();
    return;
  }
  std::vector<Point> block_centers;
  block_centers.reserve(block_indices.size());
  const FloatingPoint grid_size = submap_->getTsdfLayer().block_size();
  Point min_dimension =
      voxblox::getCenterPointFromGridIndex(block_indices.front(), grid_size);
  Point max_dimension = min_dimension;

  // Get centers and grid aligned bounding box.
  for (size_t i = 1; i < block_indices.size(); ++i) {
    block_centers.emplace_back(
        voxblox::getCenterPointFromGridIndex(block_indices[i], grid_size));
    min_dimension = min_dimension.cwiseMin(block_centers.back());
    max_dimension = max_dimension.cwiseMax(block_centers.back());
  }

  // Compute bounding sphere center and radius.
  center_ = (min_dimension + max_dimension) / 2.f;
  radius_ = 0.f;
  for (const Point& center : block_centers) {
    radius_ = std::max(radius_, (center - center_).norm());
  }
  radius_ += std::sqrt(3.f) * grid_size / 2.f;  // outermost voxel.
}

bool SubmapBoundingVolume::contains_S(const Point& point_S) const {
  // Point is expected in submap frame.
  return (center_ - point_S).norm() <= radius_;
}

bool SubmapBoundingVolume::contains_M(const Point& point_M) const {
  return contains_S(submap_->getT_S_M() * point_M);
}

bool SubmapBoundingVolume::intersects(const SubmapBoundingVolume& other) const {
  const Transformation T_S_other =
      submap_->getT_S_M() * other.submap_->getT_M_S();
  return (center_ - T_S_other * other.center_).norm() <=
         radius_ + other.radius_;
}

bool SubmapBoundingVolume::isInsidePlane_S(const Point& normal_S) const {
  // The normal is expected in submap frame.
  return center_.dot(normal_S) >= -radius_;
}

bool SubmapBoundingVolume::isInsidePlane_M(const Point& normal_S) const {
  return isInsidePlane_S(submap_->getT_S_M() * normal_S);
}

}  // namespace panoptic_mapping
