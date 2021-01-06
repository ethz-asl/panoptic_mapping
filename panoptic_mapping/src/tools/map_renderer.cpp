#include "panoptic_mapping/tools/map_renderer.h"

#include <fstream>
#include <string>

namespace panoptic_mapping {

void MapRenderer::Config::checkParams() const { checkParamConfig(camera); }

void MapRenderer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("camera", &camera);
}

MapRenderer::MapRenderer(const Config& config)
    : config_(config.checkValid()), camera_(config.camera.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Allocate range image.
  range_image_ = Eigen::MatrixXf(config_.camera.height, config_.camera.width);
}

cv::Mat MapRenderer::renderActiveSubmapIDs(const SubmapCollection& submaps,
                                           const Transformation& T_M_C) {
  // Use the mesh vertices as an approximation to render active submaps.
  // Assumes that all active submap meshes are up to date and does not perform
  // a meshing step of its own.
  range_image_.setOnes();
  range_image_ *= camera_.getConfig().max_range;
  cv::Mat result(camera_.getConfig().height, camera_.getConfig().width,
                 CV_16UC1);

  // Parse all submaps.
  for (const auto& submap_ptr : submaps) {
    // Filter out submaps.
    if (!submap_ptr->isActive()) {
      continue;
    }
    if (!camera_.submapIsInViewFrustum(*submap_ptr, T_M_C)) {
      continue;
    }

    // Project all surface points.
    const Transformation T_C_S = T_M_C.inverse() * submap_ptr->getT_M_S();
    for (const auto& surface_point : submap_ptr->getIsoSurfacePoints()) {
      const Point p_C = T_C_S * surface_point.position;
      int u, v;
      if (camera_.projectPointToImagePlane(p_C, &u, &v)) {
        float range = p_C.norm();
        if (range <= range_image_(v, u)) {
          range_image_(v, u) = range;
          result.at<u_int16_t>(v, u) =
              static_cast<u_int16_t>(submap_ptr->getID());
        }
      }
    }
  }

  return result;
}

}  // namespace panoptic_mapping
