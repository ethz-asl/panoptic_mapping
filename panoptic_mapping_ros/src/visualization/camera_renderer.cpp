#include "panoptic_mapping_ros/visualization/camera_renderer.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <minkindr_conversions/kindr_tf.h>
#include <opencv2/imgproc.hpp>
#include <tf/exceptions.h>
#include <tf/transform_datatypes.h>

#define USE_RAY_TRACING true
namespace panoptic_mapping {
void CameraRenderer::Config::checkParams() const { /* No params */
}

void CameraRenderer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
}

void CameraRenderer::Config::fromRosParam() {
  ros_namespace = rosParamNameSpace();
}

CameraRenderer::CameraRenderer(const CameraRenderer::Config& config,
                               std::shared_ptr<Globals> globals,
                               std::shared_ptr<Camera> camera,
                               bool print_config, ros::NodeHandle nh)
    : config_(config.checkValid()),
      globals_(globals),
      camera_(camera),
      nh_(nh) {
  // Print config after setting up the modes.
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
}

void CameraRenderer::renderCameraView(SubmapCollection* submaps,
                                      const tf::StampedTransform transform,
                                      cv::Mat* rendered_image) {
  auto start = std::chrono::high_resolution_clock::now();

  Transformation T_M_C;
  tf::transformTFToKindr(transform, &T_M_C);
  LOG(INFO) << "Calculating pseudo labels";

  renderImageForPose(submaps, T_M_C, rendered_image);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  LOG(INFO) << "Calculated pseudo labels in " << duration.count() << "ms";
}

void CameraRenderer::renderImageForPose(SubmapCollection* submaps,
                                        const Transformation& T_M_C,
                                        cv::Mat* rendered_image) const {
  LOG_IF(INFO, config_.verbosity > 1)
      << "checking " << submaps->size() << " submaps";
  for (const Submap& submap : *submaps) {
    // Filter out submaps.
    if (submap.getLabel() == PanopticLabel::kFreeSpace) {
      LOG_IF(INFO, config_.verbosity > 1)
          << "Cannot render image, submap is freespace.";
      continue;
    }
    if (!camera_->submapIsInViewFrustum(submap, T_M_C)) {
      LOG_IF(INFO, config_.verbosity > 1)
          << "Cannot render image, submap not in view.";
      continue;
    }

    auto cam_config_ = camera_->getConfig();
    float voxel_size_ = submap.getConfig().voxel_size;

    // Compute the 3D pointcloud from a depth image.

    const float fx_inv = 1.f / cam_config_.fx;
    const float fy_inv = 1.f / cam_config_.fy;
    Eigen::Vector3f direction;

    for (int v = 0; v < rendered_image->rows; v++) {
      for (int u = 0; u < rendered_image->cols; u++) {
        direction.x() = (static_cast<float>(u) - cam_config_.vx) * fx_inv;
        direction.y() = (static_cast<float>(v) - cam_config_.vy) * fy_inv;
        direction.z() = 1;
        direction.normalize();
        // depth_image->at<float>(v, u) = 0;
        rendered_image->at<cv::uint8_t>(v, u) = 255;
        float distance = cam_config_.min_range;  // Min distance
        while (distance <= cam_config_.max_range) {
          auto coordinates_c =
              distance * direction;  // Coordinates in camera frame
          auto coordinates_m =
              submap.getT_M_S().inverse() * T_M_C * coordinates_c;
          auto voxel =
              submap.getTsdfLayer().getVoxelPtrByCoordinates(coordinates_m);

          distance += voxel_size_;
          if (!voxel) {
            continue;
          }

          if (abs(voxel->distance) < voxel_size_) {
            const ClassVoxel* c_voxel =
                submap.getClassLayer().getVoxelPtrByCoordinates(coordinates_m);
            if (!c_voxel || !c_voxel->isObserverd() ||
                c_voxel->getBelongingID() == -1) {
              continue;
            }
            // Found intersection.
            //   depth_image->at<float>(v, u) = distance;
            rendered_image->at<cv::uint8_t>(v, u) = c_voxel->getBelongingID();
            break;
          }

          if (voxel->distance < 0) break;
        }
      }
    }
  }
}

}  // namespace panoptic_mapping
