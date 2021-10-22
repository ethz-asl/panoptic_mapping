#include "panoptic_mapping_ros/visualization/camera_renderer.h"

#include <algorithm>
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
                               bool print_config,
                               ros::NodeHandle nh)
    : config_(config.checkValid()),
      global_frame_name_("mission"),
      globals_(globals),
      camera_(camera),
      submaps_(submaps), nh_(nh) {
  // Print config after setting up the modes.
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();

  // Setup publishers.
//  nh_ = ros::NodeHandle(config_.ros_namespace);
std::cout << "CAMERA SERVICE ADVERTIESED PLEASE "<< std::endl;
  render_camera_view_srv_ = nh_.advertiseService(
      "render_camera_view", &CameraRenderer::renderCameraViewCallback, this);
  std::cout << render_camera_view_srv_.getService() << std::endl;
}

bool CameraRenderer::renderCameraViewCallback(
    panoptic_mapping_msgs::RenderCameraImage::Request& request,
    panoptic_mapping_msgs::RenderCameraImage::Response& response) {
  tf::StampedTransform transform;

  if (!request.use_provided_tf) {
    // TF not provided. Look it up.
    auto child_frame = request.sensor_frame;
    // Try to lookup the transform for the maximum wait time.
    try {
      tf_listener_.waitForTransform(global_frame_name_, child_frame, request.lookup_ts,
                                    ros::Duration(0.5));
      tf_listener_.lookupTransform(global_frame_name_, child_frame, request.lookup_ts,
                                   transform);
    } catch (tf::TransformException& ex) {
      LOG_IF(WARNING, config_.verbosity >= 0)
          << "Unable to lookup transform between '" << global_frame_name_ << "' and '"
          << child_frame << "' at time '" << request.lookup_ts << "' over '"
          << 0.5 << "s', skipping inputs. Exception: '" << ex.what() << "'.";
      return false;
    }
  } else {
    tf::transformMsgToTF(request.T_C_M, transform);
  }

  Transformation T_M_C;
  tf::transformTFToKindr(transform, &T_M_C);

  // Parse uncertainty method that should be used
  UNCERTAINTY_METHOD method;
  if (request.uncertainty_method == "uncertainty") {
    method = UNCERTAINTY_METHOD::kUNCERTAINTY;
  } else if (request.uncertainty_method == "voxel_entropy") {
    method = UNCERTAINTY_METHOD::kVOXEL_ENTROPY;
  } else if (request.uncertainty_method == "voxel_probability") {
    method = UNCERTAINTY_METHOD::kPROBABILITY;
  } else {
    LOG(WARNING) << "unknown uncertainty method " << request.uncertainty_method
                 << std::endl
                 << "Available: <uncertainty,voxel_entropy,voxel_probability>"
                 << std::endl
                 << " defaulting to uncertainty";
    method = UNCERTAINTY_METHOD::kUNCERTAINTY;
  }
  cv::Mat depth_image;
  // Initialize images width default valuse
  cv::Mat rendered_image(globals_->camera()->getConfig().height,
                         globals_->camera()->getConfig().width, CV_8UC1,
                         cv::Scalar(255));
  cv::Mat uncertainty_image(globals_->camera()->getConfig().height,
                            globals_->camera()->getConfig().width, CV_32FC1,
                            cv::Scalar(1.0f));

  if (request.use_depth) {
    // Use provided depth image to lookup voxels
    depth_image =
        cv_bridge::toCvCopy(request.depth, request.depth.encoding)->image;
    renderImageForPoseAndDepth(T_M_C, &rendered_image, &uncertainty_image,
                               &depth_image, method);
  } else {
    // Use 1000 as default depth value
    depth_image = cv::Mat(globals_->camera()->getConfig().height,
                          globals_->camera()->getConfig().width, CV_32FC1,
                          cv::Scalar(1000.0));
    renderImageForPose(T_M_C, &rendered_image, &uncertainty_image, &depth_image,
                       method);
  }
  // Assign response values
  response.depth_image =
      *cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_image)
           .toImageMsg();
  response.class_image =
      *cv_bridge::CvImage(std_msgs::Header(), "mono8", rendered_image)
           .toImageMsg();
  response.uncertainty_image =
      *cv_bridge::CvImage(std_msgs::Header(), "32FC1", uncertainty_image)
           .toImageMsg();
  return true;
}

/**
 * Get uncertainty value for a given voxel and method
 * @param voxel ClassVoxel
 * @param method Uncertainty method
 * @return Uncertainty value or -1 if uncertainty method was invalid
 */
inline float get_voxel_uncertainty_value(
    const ClassVoxelType& voxel,
    const CameraRenderer::UNCERTAINTY_METHOD method) {
  switch (method) {
    case CameraRenderer::UNCERTAINTY_METHOD::kUNCERTAINTY:
      return panoptic_mapping::classVoxelUncertainty(voxel);
    case CameraRenderer::UNCERTAINTY_METHOD::kPROBABILITY:
      return panoptic_mapping::classVoxelEntropy(voxel);
    case CameraRenderer::UNCERTAINTY_METHOD::kVOXEL_ENTROPY:
      return panoptic_mapping::classVoxelBelongingProbability(voxel);
  }
  return -1;
}

void CameraRenderer::renderImageForPoseAndDepth(
    const Transformation& T_M_C, cv::Mat* rendered_image,
    cv::Mat* uncertainty_image, const cv::Mat* depth_image,
    const UNCERTAINTY_METHOD uncertainty_method) const {
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
        auto voxel = submap.getClassLayer().getVoxelPtrByCoordinates(
            T_C_S.inverse() * p_C);
        if (voxel == nullptr || voxel->current_index < 0) continue;

        rendered_image->at<uint8_t>(i, j) = voxel->current_index;
        uncertainty_image->at<float>(i, j) =
            get_voxel_uncertainty_value(*voxel, uncertainty_method);
      }
    }
  }
}

void CameraRenderer::renderImageForPose(
    const Transformation& T_M_C, cv::Mat* rendered_image,
    cv::Mat* uncertainty_image, cv::Mat* depth_image,
    const UNCERTAINTY_METHOD uncertainty_method) const {
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

    auto cam_config_ = camera_->getConfig();
    float voxel_size_ = submap.getConfig().voxel_size;
    voxblox::BlockIndexList block_lists =
        camera_->findVisibleBlocks(submap, T_M_C, 10);
    const Transformation T_C_S = T_M_C.inverse() * submap.getT_M_S();

    for (const auto& block_idx : block_lists) {
      const TsdfBlock& block = submap.getTsdfLayer().getBlockByIndex(block_idx);
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
                get_voxel_uncertainty_value(*c_voxel, uncertainty_method), -1);
          }
        }
      }
    }
  }
}

}  // namespace panoptic_mapping
