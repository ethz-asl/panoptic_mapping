#include "panoptic_mapping/tools/map_renderer.h"

#include <fstream>
#include <string>

namespace panoptic_mapping {

void MapRenderer::Config::checkParams() const {}

void MapRenderer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("impaint_voxel_size", &impaint_voxel_size);
}

MapRenderer::MapRenderer(const Config& config, const Camera::Config& camera,
                         bool print_config)
    : config_(config.checkValid()), camera_(camera) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();

  // Allocate range image.
  range_image_ = Eigen::MatrixXf(camera.height, camera.width);
}

cv::Mat MapRenderer::render(const SubmapCollection& submaps,
                            const Transformation& T_M_C,
                            bool only_active_submaps,
                            int (*paint)(const Submap&)) {
  // Use the mesh vertices as an approximation to render active submaps.
  // Assumes that all active submap meshes are up to date and does not perform
  // a meshing step of its own. Very inefficient due to pixel duplicates.
  range_image_.setOnes();
  range_image_ *= camera_.getConfig().max_range;
  cv::Mat result = cv::Mat::ones(camera_.getConfig().height,
                                 camera_.getConfig().width, CV_32SC1) *
                   -1;

  // Parse all submaps.
  for (const Submap& submap : submaps) {
    // Filter out submaps.
    if (!submap.isActive() && only_active_submaps) {
      continue;
    }
    if (submap.getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }
    if (!camera_.submapIsInViewFrustum(submap, T_M_C)) {
      continue;
    }

    // Project all surface points.
    const Transformation T_C_S = T_M_C.inverse() * submap.getT_M_S();
    const float size_factor_x =
        camera_.getConfig().fx * submap.getTsdfLayer().voxel_size() / 2.f;
    const float size_factor_y =
        camera_.getConfig().fy * submap.getTsdfLayer().voxel_size() / 2.f;

    voxblox::BlockIndexList index_list;
    submap.getMeshLayer().getAllAllocatedMeshes(&index_list);
    for (const voxblox::BlockIndex& index : index_list) {
      for (const Point& vertex :
           submap.getMeshLayer().getMeshByIndex(index).vertices) {
        const Point p_C = T_C_S * vertex;
        int u, v;
        if (camera_.projectPointToImagePlane(p_C, &u, &v)) {
          const float range = p_C.norm();

          if (config_.impaint_voxel_size) {
            // Compensate for vertex sparsity.
            const int size_x = std::ceil(size_factor_x / p_C.z());
            const int size_y = std::ceil(size_factor_y / p_C.z());
            for (int dx = -size_x; dx <= size_x; ++dx) {
              const int u_new = u + dx;
              if (u_new < 0 || u_new >= camera_.getConfig().width) {
                continue;
              }
              for (int dy = -size_y; dy <= size_y; ++dy) {
                const int v_new = v + dy;
                if (v_new < 0 || v_new >= camera_.getConfig().height) {
                  continue;
                }
                if (range < range_image_(v_new, u_new)) {
                  range_image_(v_new, u_new) = range;
                  result.at<int>(v_new, u_new) = (*paint)(submap);
                }
              }
            }
          } else {
            if (range < range_image_(v, u)) {
              range_image_(v, u) = range;
              result.at<int>(v, u) = (*paint)(submap);
            }
          }
        }
      }
    }
  }
  return result;
}

int MapRenderer::paintSubmapID(const Submap& submap) { return submap.getID(); }

int MapRenderer::paintClass(const Submap& submap) {
  return submap.getClassID();
}

cv::Mat MapRenderer::renderActiveSubmapIDs(const SubmapCollection& submaps,
                                           const Transformation& T_M_C) {
  return render(submaps, T_M_C, true, paintSubmapID);
}

cv::Mat MapRenderer::renderActiveSubmapClasses(const SubmapCollection& submaps,
                                               const Transformation& T_M_C) {
  return render(submaps, T_M_C, true, paintClass);
}

cv::Mat MapRenderer::colorIdImage(const cv::Mat& id_image,
                                  int colors_per_revolution) {
  // Take an id_image (int) and render each ID to color using the exponential
  // color wheel for better visualization.
  cv::Mat result(id_image.rows, id_image.cols, CV_8UC3);
  if (id_image.type() != CV_32SC1) {
    LOG(WARNING) << "Input 'id_image' is not of type 'CV_32SC1', skipping.";
    return result;
  }

  id_color_map_.setItemsPerRevolution(colors_per_revolution);
  for (int u = 0; u < result.cols; ++u) {
    for (int v = 0; v < result.rows; ++v) {
      int id = id_image.at<int>(v, u);
      if (id < 0) {
        result.at<cv::Vec3b>(v, u) = cv::Vec3b{0, 0, 0};
      } else {
        const voxblox::Color color = id_color_map_.colorLookup(id);
        result.at<cv::Vec3b>(v, u) = cv::Vec3b{color.b, color.g, color.r};
      }
    }
  }
  return result;
}

}  // namespace panoptic_mapping
