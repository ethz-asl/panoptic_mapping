#include "panoptic_mapping/tracking/projective_id_tracker.h"

#include <future>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, ProjectiveIDTracker,
                                           std::shared_ptr<Globals>>
    ProjectiveIDTracker::registration_("projective");

void ProjectiveIDTracker::Config::checkParams() const {
  checkParamGT(rendering_threads, 0, "rendering_threads");
  checkParamNE(depth_tolerance, 0.f, "depth_tolerance");
  checkParamConfig(renderer);
}

void ProjectiveIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("depth_tolerance", &depth_tolerance);
  setupParam("tracking_metric", &tracking_metric);
  setupParam("match_acceptance_threshold", &match_acceptance_threshold);
  setupParam("use_class_data_for_matching", &use_class_data_for_matching);
  setupParam("min_allocation_size", &min_allocation_size);
  setupParam("rendering_threads", &rendering_threads);
  setupParam("renderer", &renderer);
}

ProjectiveIDTracker::ProjectiveIDTracker(const Config& config,
                                         std::shared_ptr<Globals> globals,
                                         bool print_config)
    : IDTrackerBase(std::move(globals)),
      config_(config.checkValid()),
      renderer_(config.renderer, globals_->camera()->getConfig(), false) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
  addRequiredInputs({InputData::InputType::kColorImage,
                     InputData::InputType::kDepthImage,
                     InputData::InputType::kSegmentationImage,
                     InputData::InputType::kValidityImage});
}

void ProjectiveIDTracker::processInput(SubmapCollection* submaps,
                                       InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Visualization.
  cv::Mat input_vis;
  std::unique_ptr<Timer> vis_timer;
  if (visualizationIsOn()) {
    vis_timer = std::make_unique<Timer>("visualization/tracking");
    Timer timer("visualization/tracking/input_image");
    input_vis = renderer_.colorIdImage(input->idImage());
    vis_timer->Pause();
  }

  // Render all submaps.
  Timer timer("tracking");
  auto t0 = std::chrono::high_resolution_clock::now();
  Timer detail_timer("tracking/compute_tracking_data");
  TrackingInfoAggregator tracking_data = computeTrackingData(submaps, input);

  // Assign the input ids to tracks or allocate new maps.
  detail_timer = Timer("tracking/match_ids");
  std::unordered_map<int, int> input_to_output;
  std::stringstream info;
  int n_matched = 0;
  int n_new = 0;
  Timer alloc_timer("tracking/allocate_submaps");
  alloc_timer.Pause();
  for (const int input_id : tracking_data.getInputIDs()) {
    int submap_id;
    bool matched = false;
    float value;
    bool any_overlap;
    std::stringstream logging_details;

    // Find matches.
    if (config_.use_class_data_for_matching || config_.verbosity >= 4) {
      std::vector<std::pair<int, float>> ids_values;
      any_overlap = tracking_data.getAllMetrics(input_id, &ids_values,
                                                config_.tracking_metric);
      // Check for classes if requested.
      if (config_.use_class_data_for_matching) {
        for (const auto& id_value : ids_values) {
          // These are ordered in decreasing overlap metric.
          if (id_value.second < config_.match_acceptance_threshold) {
            // No more matches possible.
            break;
          } else if (classesMatch(
                         input_id,
                         submaps->getSubmap(id_value.first).getClassID())) {
            // Found the best match.
            matched = true;
            submap_id = id_value.first;
            value = id_value.second;
            break;
          }
        }
      } else if (any_overlap && ids_values.front().second >
                                    config_.match_acceptance_threshold) {
        // Check only for the highest match.
        matched = true;
        submap_id = ids_values.front().first;
        value = ids_values.front().second;
      }

      // Print the matching statistics for all submaps.
      if (config_.verbosity >= 4) {
        logging_details << ". Overlap: ";
        for (const auto& id_value : ids_values) {
          logging_details << " " << id_value.first << "(" << std::fixed
                          << std::setprecision(2) << id_value.second << ")";
        }
      } else {
        logging_details << ". No overlap found.";
      }
    } else if (tracking_data.getHighestMetric(input_id, &submap_id, &value,
                                              config_.tracking_metric)) {
      // Only consider the highest metric candidate.
      if (value > config_.match_acceptance_threshold) {
        matched = true;
      }
    }

    // Allocate new submap if necessary and store tracking info.
    alloc_timer.Unpause();
    bool allocate_new_submap = tracking_data.getNumberOfInputPixels(input_id) >=
                               config_.min_allocation_size;
    if (matched) {
      n_matched++;
      input_to_output[input_id] = submap_id;
      submaps->getSubmapPtr(submap_id)->setWasTracked(true);
    } else if (allocate_new_submap) {
      n_new++;
      Submap* new_submap = allocateSubmap(input_id, submaps, input);
      if (new_submap) {
        input_to_output[input_id] = new_submap->getID();
      } else {
        input_to_output[input_id] = -1;
      }
    } else {
      // Ignore these.
      input_to_output[input_id] = -1;
    }
    alloc_timer.Pause();

    // Logging.
    if (config_.verbosity >= 3) {
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
  detail_timer.Stop();

  // Translate the id image.
  for (auto it = input->idImagePtr()->begin<int>();
       it != input->idImagePtr()->end<int>(); ++it) {
    *it = input_to_output[*it];
  }

  // Allocate free space map if required.
  alloc_timer.Unpause();
  freespace_allocator_->allocateSubmap(submaps, input);
  alloc_timer.Stop();

  // Finish.
  auto t1 = std::chrono::high_resolution_clock::now();
  timer.Stop();
  if (config_.verbosity >= 2) {
    LOG(INFO) << "Tracked IDs in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                     .count()
              << "ms, " << n_matched << " matched, " << n_new
              << " newly allocated." << info.str();
  }

  // Publish Visualization if requested.
  if (visualizationIsOn()) {
    vis_timer->Unpause();
    Timer timer("visualization/tracking/rendered");
    cv::Mat rendered_vis = renderer_.colorIdImage(
        renderer_.renderActiveSubmapIDs(*submaps, input->T_M_C()));
    timer = Timer("visualization/tracking/tracked");
    cv::Mat tracked_vis = renderer_.colorIdImage(input->idImage());
    timer.Stop();
    visualize(input_vis, "input");
    visualize(rendered_vis, "rendered");
    visualize(input->colorImage(), "color");
    visualize(tracked_vis, "tracked");
    vis_timer->Stop();
  }
}

Submap* ProjectiveIDTracker::allocateSubmap(int input_id,
                                            SubmapCollection* submaps,
                                            InputData* input) {
  LabelHandler::LabelEntry label;
  if (globals_->labelHandler()->segmentationIdExists(input_id)) {
    label = globals_->labelHandler()->getLabelEntry(input_id);
  }
  return submap_allocator_->allocateSubmap(submaps, input, input_id, label);
}

bool ProjectiveIDTracker::classesMatch(int input_id, int submap_class_id) {
  if (!globals_->labelHandler()->segmentationIdExists(input_id)) {
    // Unknown ID.
    return false;
  }
  return globals_->labelHandler()->getClassID(input_id) == submap_class_id;
}

TrackingInfoAggregator ProjectiveIDTracker::computeTrackingData(
    SubmapCollection* submaps, InputData* input) {
  // Render each active submap in parallel to collect overlap statistics.
  SubmapIndexGetter index_getter(
      globals_->camera()->findVisibleSubmapIDs(*submaps, input->T_M_C()));
  std::vector<std::future<std::vector<TrackingInfo>>> threads;
  TrackingInfoAggregator tracking_data;
  for (int i = 0; i < config_.rendering_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async,
        [this, i, &tracking_data, &index_getter, submaps,
         input]() -> std::vector<TrackingInfo> {
          // Also process the input image.
          if (i == 0) {
            tracking_data.insertInputImage(input->idImage(),
                                           input->depthImage(),
                                           globals_->camera()->getConfig());
          }
          std::vector<TrackingInfo> result;
          int index;
          while (index_getter.getNextIndex(&index)) {
            result.emplace_back(this->renderTrackingInfo(
                submaps->getSubmap(index), input->T_M_C(), input->depthImage(),
                input->idImage()));
          }
          return result;
        }));
  }

  // Join all threads.
  for (auto& thread : threads) {
    tracking_data.insertTrackingInfos(thread.get());
  }
  return tracking_data;
}

TrackingInfo ProjectiveIDTracker::renderTrackingInfo(
    const Submap& submap, const Transformation& T_M_C,
    const cv::Mat& depth_image, const cv::Mat& input_ids) const {
  // Setup.
  TrackingInfo result(submap.getID(), globals_->camera()->getConfig());
  const Transformation T_C_S = T_M_C.inverse() * submap.getT_M_S();
  const float size_factor_x = globals_->camera()->getConfig().fx *
                              submap.getTsdfLayer().voxel_size() / 2.f;
  const float size_factor_y = globals_->camera()->getConfig().fy *
                              submap.getTsdfLayer().voxel_size() / 2.f;
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
    if (!globals_->camera()->blockIsInViewFrustum(
            submap, index, T_C_S, block_size, block_diag_half)) {
      continue;
    }
    for (const Point& vertex :
         submap.getMeshLayer().getMeshByIndex(index).vertices) {
      // Project vertex and check depth value.
      const Point p_C = T_C_S * vertex;
      int u, v;
      if (!globals_->camera()->projectPointToImagePlane(p_C, &u, &v)) {
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

}  // namespace panoptic_mapping
