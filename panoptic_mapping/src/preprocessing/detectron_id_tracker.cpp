#include "panoptic_mapping/preprocessing/detectron_id_tracker.h"

#include <future>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, DetectronIDTracker,
                                           std::shared_ptr<LabelHandler>>
    DetectronIDTracker::registration_("detectron");

void DetectronIDTracker::Config::checkParams() const {
  checkParamGT(voxels_per_side, 0, "voxels_per_side");
  checkParamGT(instance_voxel_size, 0.f, "instance_voxel_size");
  checkParamGT(background_voxel_size, 0.f, "background_voxel_size");
  checkParamGT(unknown_voxel_size, 0.f, "unknown_voxel_size");
  checkParamGT(freespace_voxel_size, 0.f, "freespace_voxel_size");
  checkParamGT(rendering_threads, 0, "rendering_threads");
  checkParamNE(depth_tolerance, 0.f, "depth_tolerance");
  checkParamConfig(camera);
  checkParamConfig(renderer);
  checkParamConfig(gt_id_tracker_config);
}

void DetectronIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("instance_voxel_size", &instance_voxel_size);
  setupParam("background_voxel_size", &background_voxel_size);
  setupParam("unknown_voxel_size", &unknown_voxel_size);
  setupParam("freespace_voxel_size", &freespace_voxel_size);
  setupParam("voxels_per_side", &voxels_per_side);
  setupParam("rendering_threads", &rendering_threads);
  setupParam("depth_tolerance", &depth_tolerance);
  setupParam("tracking_metric", &tracking_metric);
  setupParam("match_acceptance_threshold", &match_acceptance_threshold);
  setupParam("debug", &debug);
  setupParam("camera_namespace", &camera_namespace);
  setupParam("camera", &camera, camera_namespace);
  setupParam("renderer", &renderer);
  renderer.camera = camera;
  setupParam("gt_id_tracker_config", &gt_id_tracker_config);
}

DetectronIDTracker::DetectronIDTracker(
    const Config& config, std::shared_ptr<LabelHandler> label_handler)
    : config_(config.checkValid()),
      IDTrackerBase(std::move(label_handler)),
      camera_(config.camera.checkValid()),
      renderer_(config.renderer.checkValid()),
      gt_tracker_(config.gt_id_tracker_config, label_handler_) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // TEST
  nh_ = ros::NodeHandle("/test");
  color_pub_ = nh_.advertise<sensor_msgs::Image>("color", 10);
  input_pub_ = nh_.advertise<sensor_msgs::Image>("input", 10);
  rendered_pub_ = nh_.advertise<sensor_msgs::Image>("rendered", 10);
  tracking_pub_ = nh_.advertise<sensor_msgs::Image>("tracking", 10);
}

void DetectronIDTracker::processImages(SubmapCollection* submaps,
                                       const Transformation& T_M_C,
                                       const cv::Mat& depth_image,
                                       const cv::Mat& color_image,
                                       cv::Mat* id_image,
                                       const DetectronLabels& labels) {
  CHECK_NOTNULL(id_image);

  // Render each active submap in parallel to collect overlap statistics.
  SubmapIndexGetter index_getter(camera_.findVisibleSubmapIDs(*submaps, T_M_C));
  std::vector<std::future<std::vector<TrackingInfo>>> threads;
  TrackingInfoAggregator tracking_data;
  for (int i = 0; i < config_.rendering_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async,
        [this, i, &tracking_data, &index_getter, submaps, &T_M_C, &depth_image,
         id_image]() -> std::vector<TrackingInfo> {
          // Also process the input image.
          if (i == 0) {
            tracking_data.insertInputImage(*id_image);
          }
          std::vector<TrackingInfo> result;
          int index;
          while (index_getter.getNextIndex(&index)) {
            result.emplace_back(this->renderTrackingInfo(
                submaps->getSubmap(index), T_M_C, depth_image, *id_image));
          }
          return result;
        }));
  }

  // Join all threads.
  for (auto& thread : threads) {
    tracking_data.insertTrackingInfos(thread.get());
  }

  // Assign the input ids to tracks or allocate new maps.
  std::unordered_map<int, int> input_to_output;
  std::stringstream info;
  for (const int input_id : tracking_data.getInputIDs()) {
    // Print the matching statistics if required.
    if (config_.verbosity >= 4) {
      std::vector<std::pair<int, float>> mapid_ious;
      if (tracking_data.getAllMetrics(input_id, &mapid_ious)) {
        info << "\n  " << input_id << ":";
        for (const auto& id_io : mapid_ious) {
          info << " " << id_io.first << "(" << std::fixed
               << std::setprecision(2) << id_io.second << ")";
        }
      } else {
        info << "\n  " << input_id << ": new";
      }
    }
  }
  if (config_.verbosity >= 4) {
    LOG(INFO) << "ID Tracking overlap statistics: input: submap(IoU)"
              << info.str();
  }

  // TEST Publish Visualization.
  if (config_.debug) {
    cv::Mat rendered_ids;
    rendered_ids = renderer_.renderActiveSubmapIDs(*submaps, T_M_C);
    cv::Mat input_vis = renderer_.colorIdImage(*id_image);
    cv::Mat rendered_vis = renderer_.colorIdImage(rendered_ids);
    gt_tracker_.processImages(submaps, T_M_C, depth_image, color_image,
                              id_image);
    cv::Mat tracked_vis = renderer_.colorIdImage(*id_image);
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    auto enc = sensor_msgs::image_encodings::BGR8;
    color_pub_.publish(
        cv_bridge::CvImage(header, enc, color_image).toImageMsg());
    input_pub_.publish(cv_bridge::CvImage(header, enc, input_vis).toImageMsg());
    rendered_pub_.publish(
        cv_bridge::CvImage(header, enc, rendered_vis).toImageMsg());
    tracking_pub_.publish(
        cv_bridge::CvImage(header, enc, tracked_vis).toImageMsg());
  }
}

TrackingInfo DetectronIDTracker::renderTrackingInfo(
    const Submap& submap, const Transformation& T_M_C,
    const cv::Mat& depth_image, const cv::Mat& input_ids) const {
  // Setup.
  TrackingInfo result(submap.getID(), camera_.getConfig());
  const Transformation T_C_S = T_M_C.inverse() * submap.getT_M_S();
  const float size_factor_x =
      camera_.getConfig().fx * submap.getTsdfLayer().voxel_size() / 2.f;
  const float size_factor_y =
      camera_.getConfig().fy * submap.getTsdfLayer().voxel_size() / 2.f;
  const float block_size = submap.getTsdfLayer().block_size();
  const FloatingPoint block_diag_half = std::sqrt(3.0f) * block_size / 2.0f;
  const float depth_tolerance =
      config_.depth_tolerance > 0
          ? config_.depth_tolerance
          : -config_.depth_tolerance * submap.getTsdfLayer().voxel_size();

  // Parse all blocks.
  voxblox::BlockIndexList index_list;
  submap.getMeshLayer().getAllAllocatedMeshes(&index_list);
  for (const voxblox::BlockIndex& index : index_list) {
    if (!camera_.blockIsInViewFrustum(submap, index, T_C_S, block_size,
                                      block_diag_half)) {
      continue;
    }
    for (const Point& vertex :
         submap.getMeshLayer().getMeshByIndex(index).vertices) {
      // Project vertex and check depth value.
      const Point p_C = T_C_S * vertex;
      int u, v;
      if (!camera_.projectPointToImagePlane(p_C, &u, &v)) {
        continue;
      }
      if (std::abs(depth_image.at<float>(v, u) - p_C.z()) >= depth_tolerance) {
        continue;
      }

      // Compensate for vertex sparsity.
      const int size_x = std::ceil(size_factor_x / p_C.z());
      const int size_y = std::ceil(size_factor_y / p_C.z());
      result.insertRenderedPoint(u, v, size_x, size_y);
    }
  }
  result.evaluate(input_ids);
  return result;
}

void DetectronIDTracker::processImages(SubmapCollection* submaps,
                                       const Transformation& T_M_C,
                                       const cv::Mat& depth_image,
                                       const cv::Mat& color_image,
                                       cv::Mat* id_image) {
  LOG(WARNING) << "The detectron id tracker can only be used with labels.";
}

void DetectronIDTracker::processPointcloud(SubmapCollection* submaps,
                                           const Transformation& T_M_C,
                                           const Pointcloud& pointcloud,
                                           const Colors& colors,
                                           std::vector<int>* ids) {
  LOG(WARNING) << "The detectron id tracker can only be used with labels.";
}

int DetectronIDTracker::allocateSubmap(int detectron_id,
                                       SubmapCollection* submaps,
                                       const DetectronLabels& labels) {
  // Check whether the instance code is known.
  auto it = labels.find(detectron_id);
  PanopticLabel pan_label = PanopticLabel::kUnknown;
  if (it == labels.end()) {
    auto error_it = unknown_ids.find(detectron_id);
    if (error_it == unknown_ids.end()) {
      unknown_ids[detectron_id] = 1;
    } else {
      unknown_ids[detectron_id] += 1;
    }
  } else {
    if (it->second.is_thing) {
      pan_label = PanopticLabel::kInstance;
    } else {
      pan_label = PanopticLabel::kBackground;
    }
  }

  // Allocate new submap.
  Submap::Config cfg;
  cfg.voxels_per_side = config_.voxels_per_side;
  switch (pan_label) {
    case PanopticLabel::kInstance: {
      cfg.voxel_size = config_.instance_voxel_size;
      break;
    }
    case PanopticLabel::kBackground: {
      cfg.voxel_size = config_.background_voxel_size;
      break;
    }
    case PanopticLabel::kFreeSpace: {
      cfg.voxel_size = config_.freespace_voxel_size;
      break;
    }
    case PanopticLabel::kUnknown: {
      cfg.voxel_size = config_.unknown_voxel_size;
      break;
    }
  }
  Submap* new_submap = submaps->createSubmap(cfg);
  new_submap->setLabel(pan_label);
  // TODO(schmluk): add proper data.
  if (pan_label != PanopticLabel::kUnknown) {
    new_submap->setClassID(it->second.category_id);
  } else {
    // new_submap->setClassID(-1);
  }
  // new_submap->setInstanceID(new_instance);
  // new_submap->setName(label_handler_->getName(new_instance));
  return new_submap->getID();
}

void DetectronIDTracker::allocateFreeSpaceSubmap(SubmapCollection* submaps) {
  if (submaps->getActiveFreeSpaceSubmapID() >= 0) {
    // Currently only allocate one free space submap in the beginning.
    return;
  }

  // Create a new freespace submap.
  Submap::Config config;
  config.voxels_per_side = config_.voxels_per_side;
  config.voxel_size = config_.freespace_voxel_size;
  Submap* space_submap = submaps->createSubmap(config);
  space_submap->setLabel(PanopticLabel::kFreeSpace);
  space_submap->setInstanceID(-1);  // Will never appear in a seg image.
  space_submap->setName("FreeSpace");
  submaps->setActiveFreeSpaceSubmapID(space_submap->getID());
}

void DetectronIDTracker::printAndResetWarnings() {
  for (auto it : unknown_ids) {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Encountered " << it.second
        << " occurences of unknown segmentation ID '" << it.first << "'.";
  }
  unknown_ids.clear();
}

}  // namespace panoptic_mapping
