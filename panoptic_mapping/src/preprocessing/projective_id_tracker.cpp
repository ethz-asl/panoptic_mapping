#include "panoptic_mapping/preprocessing/projective_id_tracker.h"

#include <future>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, ProjectiveIDTracker,
                                           std::shared_ptr<LabelHandler>>
    ProjectiveIDTracker::registration_("projective");

void ProjectiveIDTracker::Config::checkParams() const {
  checkParamGT(voxels_per_side, 0, "voxels_per_side");
  checkParamGT(instance_voxel_size, 0.f, "instance_voxel_size");
  checkParamGT(background_voxel_size, 0.f, "background_voxel_size");
  checkParamGT(unknown_voxel_size, 0.f, "unknown_voxel_size");
  checkParamGT(freespace_voxel_size, 0.f, "freespace_voxel_size");
  checkParamGT(rendering_threads, 0, "rendering_threads");
  checkParamNE(depth_tolerance, 0.f, "depth_tolerance");
  checkParamConfig(camera);
  checkParamConfig(renderer);
}

void ProjectiveIDTracker::Config::setupParamsAndPrinting() {
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
  setupParam("min_allocation_size", &min_allocation_size);
  setupParam("input_is_mesh_id", &input_is_mesh_id);
  setupParam("camera_namespace", &camera_namespace);
  setupParam("camera", &camera, camera_namespace);
  setupParam("renderer", &renderer);
  renderer.camera = camera;
}

ProjectiveIDTracker::ProjectiveIDTracker(
    const Config& config, std::shared_ptr<LabelHandler> label_handler)
    : config_(config.checkValid()),
      IDTrackerBase(std::move(label_handler)),
      camera_(config.camera.checkValid()),
      renderer_(config.renderer.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void ProjectiveIDTracker::processInput(SubmapCollection* submaps,
                                       InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Visualization.
  cv::Mat input_vis;
  if (visualizationIsOn()) {
    input_vis = renderer_.colorIdImage(*(input->idImage()));
  }

  // Render each active submap in parallel to collect overlap statistics.
  auto t0 = std::chrono::high_resolution_clock::now();
  SubmapIndexGetter index_getter(
      camera_.findVisibleSubmapIDs(*submaps, input->T_M_C()));
  std::vector<std::future<std::vector<TrackingInfo>>> threads;
  TrackingInfoAggregator tracking_data;
  for (int i = 0; i < config_.rendering_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async,
        [this, i, &tracking_data, &index_getter, submaps,
         input]() -> std::vector<TrackingInfo> {
          // Also process the input image.
          if (i == 0) {
            tracking_data.insertInputImage(
                *(input->idImage()), input->depthImage(), camera_.getConfig());
          }
          std::vector<TrackingInfo> result;
          int index;
          while (index_getter.getNextIndex(&index)) {
            result.emplace_back(this->renderTrackingInfo(
                submaps->getSubmap(index), input->T_M_C(), input->depthImage(),
                *(input->idImage())));
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
  int n_matched = 0;
  int n_new = 0;
  for (const int input_id : tracking_data.getInputIDs()) {
    int submap_id;
    bool matched = false;
    float value;
    bool any_overlap;
    std::stringstream logging_details;

    if (config_.verbosity < 4) {
      any_overlap = tracking_data.getHighestMetric(input_id, &submap_id, &value,
                                                   config_.tracking_metric);
      if (any_overlap) {
        if (value > config_.match_acceptance_threshold) {
          matched = true;
        }
      }
    } else {
      // Print the matching statistics for al submaps.
      std::vector<std::pair<int, float>> ids_values;
      any_overlap = tracking_data.getAllMetrics(input_id, &ids_values,
                                                config_.tracking_metric);
      if (any_overlap) {
        if (ids_values.front().second > config_.match_acceptance_threshold) {
          matched = true;
          submap_id = ids_values.front().first;
          value = ids_values.front().second;
        }
        logging_details << ". Overlap: ";
        for (const auto& id_value : ids_values) {
          logging_details << " " << id_value.first << "(" << std::fixed
                          << std::setprecision(2) << id_value.second << ")";
        }
      } else {
        logging_details << ". No overlap found.";
      }
    }

    // Allocate new submap if necessary and store tracking info.
    bool allocate_new_submap = tracking_data.getNumberOfInputPixels(input_id) >=
                               config_.min_allocation_size;
    if (matched) {
      n_matched++;
      input_to_output[input_id] = submap_id;
    } else if (allocate_new_submap) {
      n_new++;
      input_to_output[input_id] = allocateSubmap(input_id, submaps);
    } else {
      // Ignore these
      input_to_output[input_id] = -1;
    }

    // Logging.
    if (config_.verbosity >= 2) {
      if (matched) {
        info << "\n  " << input_id << "->" << submap_id << " (" << std::fixed
             << std::setprecision(2) << value << ")";
      } else {
        if (allocate_new_submap) {
          info << "\n  " << input_id << "->" << input_to_output[input_id]
               << " [new]";
        } else {
          info << "\n  " << input_id << " [ignored]";
        }
        if (any_overlap) {
          info << " (" << std::fixed << std::setprecision(2) << value << ")";
        }
      }
      info << logging_details.str();
    }
  }

  // Translate the id image.
  for (auto it = input->idImage()->begin<int>();
       it != input->idImage()->end<int>(); ++it) {
    *it = input_to_output[*it];
  }

  // Allocate free space map if required.
  allocateFreeSpaceSubmap(submaps);

  // Finish.
  auto t1 = std::chrono::high_resolution_clock::now();
  if (config_.verbosity >= 2) {
    LOG(INFO) << "Tracked IDs in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                     .count()
              << "ms, " << n_matched << " matched, " << n_new
              << " newly allocated." << info.str();
  }

  // Publish Visualization if requested.
  if (visualizationIsOn()) {
    cv::Mat rendered_vis = renderer_.colorIdImage(
        renderer_.renderActiveSubmapIDs(*submaps, input->T_M_C()));
    cv::Mat tracked_vis = renderer_.colorIdImage(*(input->idImage()));
    visualize(input_vis, "input");
    visualize(rendered_vis, "rendered");
    visualize(input->colorImage(), "color");
    visualize(tracked_vis, "tracked");
  }
}

TrackingInfo ProjectiveIDTracker::renderTrackingInfo(
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
  result.evaluate(input_ids, depth_image);
  return result;
}

int ProjectiveIDTracker::allocateSubmap(int instance_id,
                                        SubmapCollection* submaps) {
  // Lookup the labels associated with the input_id
  PanopticLabel label = PanopticLabel::kUnknown;
  if (config_.input_is_mesh_id) {
    instance_id = label_handler_->getSegmentationIdFromMeshId(instance_id);
    // NOTE(schmluk): If not found returns -1, which shouldn't be in the labels.
  }
  if (label_handler_->segmentationIdExists(instance_id)) {
    label = label_handler_->getPanopticLabel(instance_id);
  }

  // Allocate new submap.
  Submap::Config cfg;
  cfg.voxels_per_side = config_.voxels_per_side;
  switch (label) {
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

  // Take all available information from the ground truth instance ids.
  new_submap->setInstanceID(instance_id);
  new_submap->setLabel(label);
  if (label != PanopticLabel::kUnknown) {
    new_submap->setClassID(label_handler_->getClassID(instance_id));
    new_submap->setName(label_handler_->getName(instance_id));
  }
  return new_submap->getID();
}

void ProjectiveIDTracker::allocateFreeSpaceSubmap(SubmapCollection* submaps) {
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

}  // namespace panoptic_mapping
