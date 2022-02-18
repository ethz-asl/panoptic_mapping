#include "panoptic_mapping_ros/visualization/camera_renderer.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
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
                               std::shared_ptr<SubmapCollection> submaps,
                               bool print_config, ros::NodeHandle nh)
    : config_(config.checkValid()),
      globals_(globals),
      camera_(camera),
      submaps_(submaps),
      nh_(nh) {
  // Print config after setting up the modes.
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();

  // Setup publishers.
  render_camera_view_srv_ = nh_.advertiseService(
      "render_camera_view", &CameraRenderer::renderCameraViewCallback, this);
  std::cout << render_camera_view_srv_.getService() << std::endl;
}

bool CameraRenderer::renderCameraViewCallback(
    panoptic_mapping_msgs::RenderCameraImage::Request& request,
    panoptic_mapping_msgs::RenderCameraImage::Response& response) {
  tf::StampedTransform transform;
  auto start = std::chrono::high_resolution_clock::now();
  tf::transformMsgToTF(request.T_C_M, transform);

  Transformation T_M_C;
  tf::transformTFToKindr(transform, &T_M_C);
  LOG(INFO) << "Calculating pseudo labels";
  cv::Mat depth_image;
  // Initialize images width default valuse
  cv::Mat rendered_image(globals_->camera()->getConfig().height,
                         globals_->camera()->getConfig().width, CV_8UC1,
                         cv::Scalar(255));
  cv::Mat uncertainty_image(globals_->camera()->getConfig().height,
                            globals_->camera()->getConfig().width, CV_32FC1,
                            cv::Scalar(1.0f));

  // Use 1000 as default depth value
  depth_image = cv::Mat(globals_->camera()->getConfig().height,
                        globals_->camera()->getConfig().width, CV_32FC1,
                        cv::Scalar(1000.0));
  renderImageForPose(T_M_C, &rendered_image);
  // Assign response values
  response.depth_image =
      *cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_image)
           .toImageMsg();
  response.class_image =
      *cv_bridge::CvImage(std_msgs::Header(), "mono8", rendered_image)
           .toImageMsg();
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  LOG(INFO) << "Calculated pseudo labels in " << duration.count() << "ms";
  return true;
}

void CameraRenderer::renderImageForPoseAndDepth(
    const Transformation& T_M_C, cv::Mat* rendered_image,
    cv::Mat* uncertainty_image, const cv::Mat* depth_image) const {
  auto cam_config_ = camera_->getConfig();
  cv::Mat vertex_map = camera_->computeVertexMap(*depth_image);

  for (const Submap& submap : *submaps_) {
    // Filter out submaps.
    if (!submap.isActive()) {
      continue;
    }
    if (submap.getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }
    if (!camera_->submapIsInViewFrustum(submap, T_M_C)) {
      continue;
    }

    const Transformation T_C_S = T_M_C.inverse() * submap.getT_M_S();

    for (int i = 0; i < cam_config_.height; i++) {
      for (int j = 0; j < cam_config_.width; j++) {
        // Project all voxels
        auto vertex = vertex_map.at<cv::Vec3f>(i, j);
        Eigen::Matrix<FloatingPoint, 3, 1> p_C(vertex(0), vertex(1), vertex(2));
        const ClassVoxel* voxel =
            submap.getClassLayer().getVoxelPtrByCoordinates(T_C_S.inverse() *
                                                            p_C);
        if (voxel == nullptr || !voxel->isObserverd() ||
            voxel->getBelongingID() < 0)
          continue;

        rendered_image->at<uint8_t>(i, j) = voxel->getBelongingID();
      }
    }
  }
}

void CameraRenderer::renderImageForPose(const Transformation& T_M_C,
                                        cv::Mat* rendered_image) const {
  LOG_IF(INFO, config_.verbosity > 1)
      << "checking " << submaps_->size() << " submaps";
  for (const Submap& submap : *submaps_) {
    // Filter out submaps.
    if (!submap.isActive()) {
      LOG_IF(INFO, config_.verbosity > 1)
          << "Cannot render image, submap inactive.";
      continue;
    }
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

    if (USE_RAY_TRACING) {
      LOG_IF(INFO, config_.verbosity > 1) << "rendering with ray-tracing";
      continue;
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
            if (!voxel) continue;
            LOG_IF(INFO, config_.verbosity > 1) << "found voxel";
            continue;

            if (abs(voxel->distance) < voxel_size_) {
              const ClassVoxel* c_voxel =
                  submap.getClassLayer().getVoxelPtrByCoordinates(
                      coordinates_m);
              if (!c_voxel || !c_voxel->isObserverd() ||
                  c_voxel->getBelongingID() == -1) {
                LOG_IF(INFO, config_.verbosity > 1) << "not observed";
                continue;
              }
              // Found intersection.
              //   depth_image->at<float>(v, u) = distance;
              rendered_image->at<cv::uint8_t>(v, u) = c_voxel->getBelongingID();
              break;
            }
          }
        }
      }

    } else {
      /*auto cam_config_ = camera_->getConfig();
      float voxel_size_ = submap.getConfig().voxel_size;
      voxblox::BlockIndexList block_lists =
          camera_->findVisibleBlocks(submap, T_M_C, 10);
      const Transformation T_C_S = T_M_C.inverse() * submap.getT_M_S();

      for (const auto& block_idx : block_lists) {
        const TsdfBlock& block =
            submap.getTsdfLayer().getBlockByIndex(block_idx);
        const ClassBlock* c_block =
            &submap.getClassLayer().getBlockByIndex(block_idx);
        for (size_t i = 0; i < block.num_voxels(); ++i) {
          const TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
          if (abs(voxel.distance) < voxel_size_) {
            // Found surface voxel, Project it.
            int u, v;
            auto p_C = T_C_S * block.computeCoordinatesFromLinearIndex(i);
            if (p_C.z() <= 0.0) {
              continue;
            }

            if (!camera_->projectPointToImagePlane(p_C, &u, &v) || u < 0 ||
                u >= cam_config_.width || v < 0 || v >= cam_config_.height) {
              continue;
            }

            if (depth_image->at<float>(v, u) <= p_C.z()) {
              continue;  // Already have label for this point
            }

            const ClassVoxelType* c_voxel = &c_block->getVoxelByLinearIndex(i);
            if (c_voxel->current_index >= 0) {
              // Draw a rectangle of voxel size and project it.
              // Otherwise we end up with a lot of small points and empty space
              int u_min, v_min;
              Eigen::Matrix<float, 3, 1> p_C_min;
              p_C_min.x() = p_C.x() - voxel_size_ / 2;
              p_C_min.y() = p_C.y() - voxel_size_ / 2;
              p_C_min.z() = p_C.z() - voxel_size_ / 2;

              int u_max, v_max;
              Eigen::Matrix<float, 3, 1> p_C_max;
              p_C_max.x() = p_C.x() + voxel_size_ / 2;
              p_C_max.y() = p_C.y() + voxel_size_ / 2;
              p_C_max.z() = p_C.z() - voxel_size_ / 2;
              // Project corners of rectangle
              camera_->projectPointToImagePlane(p_C_min, &u_min, &v_min);
              camera_->projectPointToImagePlane(p_C_max, &u_max, &v_max);
              // Make sure it fits into image
              u_min = std::max(0, u_min);
              v_min = std::min(cam_config_.height - 1, v_min);
              u_max = std::max(0, u_max);
              v_max = std::min(cam_config_.width - 1, v_max);
              // Draw
              cv::rectangle(*rendered_image, cv::Point(u_min, v_min),
                            cv::Point(u_max, v_max),
                            cv::Scalar(c_voxel->current_index), -1);
              cv::rectangle(*depth_image, cv::Point(u_min, v_min),
                            cv::Point(u_max, v_max), p_C.z(), -1);
              cv::rectangle(
                  *uncertainty_image, cv::Point(u_min, v_min),
                  cv::Point(u_max, v_max),
                  get_voxel_uncertainty_value(*c_voxel, uncertainty_method),
                  -1);
            }
          }
        }
      }*/
    }
  }
}

}  // namespace panoptic_mapping
