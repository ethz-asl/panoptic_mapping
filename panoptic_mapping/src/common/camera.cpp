#include "panoptic_mapping/common/camera.h"

namespace panoptic_mapping {

void Camera::Config::checkParams() const {
  checkParamGT(width, 0, "width");
  checkParamGT(height, 0, "height");
  checkParamGT(vx, 0.f, "vx");
  checkParamGT(vy, 0.f, "vy");
  checkParamGT(fx, 0.f, "fx");
  checkParamGT(fy, 0.f, "fy");
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

bool Camera::projectPointToImagePlane(const Point& p_C, float* u,
                                      float* v) const {
  if (p_C.z() < config_.min_range) {
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

}  // namespace panoptic_mapping
