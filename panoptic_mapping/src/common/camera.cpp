#include "panoptic_mapping/common/camera.h"

#include <unordered_map>
#include <vector>

namespace panoptic_mapping {

void Camera::Config::checkParams() const {
  checkParamGT(width, 0, "width");
  checkParamGT(height, 0, "height");
  checkParamGT(vx, 0.f, "vx");
  checkParamGT(vy, 0.f, "vy");
  checkParamGT(fx, 0.f, "fx");
  checkParamGT(fy, 0.f, "fy");
  checkParamGT(min_range, 0.f, "min_range");
  checkParamCond(max_range > min_range,
                 "'max_range' is expected > 'min_range'.");
  checkParamCond(vx <= width, "'vx' is expected <= 'width'.");
  checkParamCond(vy <= height, "'vy' is expected <= 'height'.");
}

void Camera::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("width", &width);
  setupParam("height", &height);
  setupParam("vx", &vx);
  setupParam("vy", &vy);
  setupParam("fx", &fx);
  setupParam("fy", &fy);
  setupParam("max_range", &max_range);
  setupParam("min_range", &min_range);
}

Camera::Camera(const Config& config) : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Pre-compute the view frustum (top, right, bottom, left, plane normals).
  const float scale_factor = config_.fy / config_.fx;
  Eigen::Vector3f p1(-config_.vx, -config_.vy * scale_factor, config_.fx);
  Eigen::Vector3f p2(config_.width - config_.vx, -config_.vy * scale_factor,
                     config_.fx);
  Eigen::Vector3f normal = p1.cross(p2);
  view_frustum_.push_back(normal.normalized());
  p1 =
      Eigen::Vector3f(config_.width - config_.vx,
                      (config_.height - config_.vy) * scale_factor, config_.fx);
  normal = p2.cross(p1);
  view_frustum_.push_back(normal.normalized());
  p2 = Eigen::Vector3f(
      -config_.vx, (config_.height - config_.vy) * scale_factor, config_.fx);
  normal = p1.cross(p2);
  view_frustum_.push_back(normal.normalized());
  p1 = Eigen::Vector3f(-config_.vx, -config_.vy * scale_factor, config_.fx);
  normal = p2.cross(p1);
  view_frustum_.push_back(normal.normalized());
}

bool Camera::pointIsInViewFrustum(const Point& point_C,
                                  float inflation_distance) const {
  if (point_C.z() < -inflation_distance) {
    return false;
  }
  if (point_C.norm() > config_.max_range + inflation_distance) {
    return false;
  }
  for (const Point& view_frustum_plane : view_frustum_) {
    if (point_C.dot(view_frustum_plane) < -inflation_distance) {
      return false;
    }
  }
  return true;
}

bool Camera::submapIsInViewFrustum(const Submap& submap,
                                   const Transformation& T_M_C) const {
  const float radius = submap.getBoundingVolume().getRadius();
  const Point center_C = T_M_C.inverse() * submap.getT_M_S() *
                         submap.getBoundingVolume().getCenter();
  return pointIsInViewFrustum(center_C, radius);
}

bool Camera::blockIsInViewFrustum(const Submap& submap,
                                  const voxblox::BlockIndex& block_index,
                                  const Transformation& T_M_C) const {
  const Transformation T_C_S =
      T_M_C.inverse() * submap.getT_M_S();  // p_C = T_C_M * T_M_S * p_S
  const FloatingPoint block_size = submap.getTsdfLayer().block_size();
  const FloatingPoint block_diag_half = std::sqrt(3.0f) * block_size / 2.0f;
  return blockIsInViewFrustum(submap, block_index, T_C_S, block_size,
                              block_diag_half);
}

bool Camera::blockIsInViewFrustum(const Submap& submap,
                                  const voxblox::BlockIndex& block_index,
                                  const Transformation& T_C_S, float block_size,
                                  float block_diag_half) const {
  auto& block = submap.getTsdfLayer().getBlockByIndex(block_index);
  const Point p_C =
      T_C_S * (block.origin() +
               Point(1, 1, 1) * block_size / 2.0);  // center point of the block
  return pointIsInViewFrustum(p_C, block_diag_half);
}

bool Camera::projectPointToImagePlane(const Point& p_C, float* u,
                                      float* v) const {
  if (p_C.z() < config_.min_range || p_C.z() > config_.max_range) {
    return false;
  }
  // All values are ceiled and floored to guarantee that the resulting points
  // will be valid for any integer conversion.
  CHECK_NOTNULL(u);
  *u = p_C.x() * config_.fx / p_C.z() + config_.vx;
  if (std::ceil(*u) >= config_.width || std::floor(*u) < 0) {
    return false;
  }
  CHECK_NOTNULL(v);
  *v = p_C.y() * config_.fy / p_C.z() + config_.vy;
  if (std::ceil(*v) >= config_.height || std::floor(*v) < 0) {
    return false;
  }
  return true;
}

bool Camera::projectPointToImagePlane(const Point& p_C, int* u, int* v) const {
  if (p_C.z() <= config_.min_range) {
    return false;
  }
  CHECK_NOTNULL(u);
  CHECK_NOTNULL(v);
  *u = std::round(p_C.x() * config_.fx / p_C.z() + config_.vx);
  if (*u >= config_.width || *u < 0) {
    return false;
  }
  *v = std::round(p_C.y() * config_.fy / p_C.z() + config_.vy);
  if (*v >= config_.height || *v < 0) {
    return false;
  }
  return true;
}

std::vector<int> Camera::findVisibleSubmapIDs(const SubmapCollection& submaps,
                                              const Transformation& T_M_C,
                                              bool only_active_submaps,
                                              bool include_freespace) const {
  std::vector<int> result;
  for (const Submap& submap : submaps) {
    if (!submap.isActive() && only_active_submaps) {
      continue;
    }
    if (submap.getLabel() == PanopticLabel::kFreeSpace && !include_freespace) {
      continue;
    }
    if (!submapIsInViewFrustum(submap, T_M_C)) {
      continue;
    }
    result.push_back(submap.getID());
  }
  return result;
}

voxblox::BlockIndexList Camera::findVisibleBlocks(
    const Submap& submap, const Transformation& T_M_C) const {
  // Setup.
  voxblox::BlockIndexList result;
  voxblox::BlockIndexList all_blocks;
  submap.getTsdfLayer().getAllAllocatedBlocks(&all_blocks);
  const Transformation T_C_S =
      T_M_C.inverse() * submap.getT_M_S();  // p_C = T_C_M * T_M_S * p_S
  const FloatingPoint block_size = submap.getTsdfLayer().block_size();
  const FloatingPoint block_diag_half = std::sqrt(3.0f) * block_size / 2.0f;

  // Iterate through all blocks.
  for (auto& index : all_blocks) {
    auto& block = submap.getTsdfLayer().getBlockByIndex(index);
    const Point p_C =
        T_C_S * (block.origin() + Point(1, 1, 1) * block_size /
                                      2.0);  // center point of the block

    if (pointIsInViewFrustum(p_C, block_diag_half)) {
      result.push_back(index);
    }
  }
  return result;
}

std::unordered_map<int, voxblox::BlockIndexList> Camera::findVisibleBlocks(
    const SubmapCollection& submaps, const Transformation& T_M_C,
    bool only_active_submaps) const {
  std::unordered_map<int, voxblox::BlockIndexList> result;
  for (const Submap& submap : submaps) {
    if (!submap.isActive() && only_active_submaps) {
      continue;
    }
    if (!submapIsInViewFrustum(submap, T_M_C)) {
      continue;
    }
    voxblox::BlockIndexList block_list = findVisibleBlocks(submap, T_M_C);
    if (!block_list.empty()) {
      result[submap.getID()] = block_list;
    }
  }
  return result;
}

cv::Mat Camera::computeVertexMap(const cv::Mat& depth_image) const {
  // Compute the 3D pointcloud from a depth image.
  cv::Mat vertices(depth_image.size(), CV_32FC3);
  const float fx_inv = 1.f / config_.fx;
  const float fy_inv = 1.f / config_.fy;
  for (int v = 0; v < depth_image.rows; v++) {
    for (int u = 0; u < depth_image.cols; u++) {
      cv::Vec3f& vertex = vertices.at<cv::Vec3f>(v, u);  // x, y, z
      vertex[2] = depth_image.at<float>(v, u);
      vertex[0] = (static_cast<float>(u) - config_.vx) * vertex[2] * fx_inv;
      vertex[1] = (static_cast<float>(v) - config_.vy) * vertex[2] * fy_inv;
    }
  }
  return vertices;
}

}  // namespace panoptic_mapping
