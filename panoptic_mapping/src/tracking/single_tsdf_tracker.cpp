#include "panoptic_mapping/tracking/single_tsdf_tracker.h"

#include <memory>
#include <unordered_map>
#include <utility>

#include "panoptic_mapping/tracking/tracking_info.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, SingleTSDFTracker,
                                           std::shared_ptr<Globals>>
    SingleTSDFTracker::registration_("single_tsdf");

void SingleTSDFTracker::Config::checkParams() const {
  checkParamConfig(submap);
  checkParamConfig(renderer);
}

void SingleTSDFTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("tracking_metric", &tracking_metric);
  setupParam("submap", &submap);
  setupParam("use_detectron", &use_detectron);
  setupParam("use_detectron_panoptic", &use_detectron_panoptic);
  setupParam("use_class_for_instance_tracking",
             &use_class_for_instance_tracking);
  setupParam("renderer", &renderer);
  setupParam("min_new_instance_size", &min_new_instance_size);
  setupParam("match_acceptance_threshold", &match_acceptance_threshold);
}

SingleTSDFTracker::SingleTSDFTracker(const Config& config,
                                     std::shared_ptr<Globals> globals)
    : IDTrackerBase(std::move(globals)),
      config_(config.checkValid()),
      renderer_(config.renderer, globals_->camera()->getConfig(), false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  addRequiredInput(InputData::InputType::kColorImage);
  addRequiredInput(InputData::InputType::kDepthImage);
  if (config_.submap.useClassLayer()) {
    addRequiredInput(InputData::InputType::kSegmentationImage);
  }
  if (config_.use_detectron) {
    addRequiredInput(InputData::InputType::kDetectronLabels);
  }
}

void SingleTSDFTracker::processInput(SubmapCollection* submaps,
                                     InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));

  // Check whether the map is already allocated.
  if (!is_setup_) {
    setup(submaps);
  }

  // Visualization
  cv::Mat input_vis;
  if (visualizationIsOn()) {
    input_vis = renderer_.colorIdImage(input->idImage());
  }

  // Use only semantic information
  if (config_.use_detectron && !config_.use_detectron_panoptic) {
    parseDetectronClasses(input);
  }

  // Use both semantic and instance level information
  if (config_.use_detectron_panoptic) {
    // Convert the IDs in the ID image to panoptic ids
    parseDetectronPanopticLabels(input);

    int n_matched = 0;
    int n_new = 0;

    // Mapping from input ids to track ids i.e. existing instances
    std::unordered_map<int, int> input_to_output;

    // Render all the panoptic entities visible in the map
    TrackingInfoAggregator tracking_data = computeTrackingData(submaps, input);
    std::set<int> matched_ids;
    for (const int input_id : tracking_data.getInputIDs()) {
      bool matched = false;
      int matched_id = 0;
      float score = 0.f;

      const auto input_class_id = input_id / kPanopticLabelDivisor_;
      const auto input_instance_id = input_id % kPanopticLabelDivisor_;

      // Stuff segments don't need to be matched
      if (input_instance_id == 0) {
        input_to_output[input_id] = input_id;
      }

      // Compute IoU with all rendered segments
      std::vector<std::pair<int, float>> ids_values;
      bool any_overlap = tracking_data.getAllMetrics(input_id, &ids_values,
                                                     config_.tracking_metric);
      if (!any_overlap) {
        continue;
      }

      for (const auto& id_value : ids_values) {
        if (id_value.second < config_.match_acceptance_threshold) {
          // No more matches possible as matches are ordered in decreasing order
          break;
        }

        // Check that classes match, if enabled
        const auto class_id = id_value.first / kPanopticLabelDivisor_;
        if (class_id != input_class_id &&
            config_.use_class_for_instance_tracking) {
          continue;
        }

        // Make sure rendered and predicted segments are matched 1-to-1
        if (matched_ids.find(id_value.first) != matched_ids.end()) {
          continue;
        }

        // Found the best match.
        matched = true;
        matched_id = id_value.first;
        score = id_value.second;
        break;
      }

      if (matched) {
        ++n_matched;
        input_to_output[input_id] = matched_id;
        matched_ids.insert(matched_id);
      } else {
        // Filter segments that are too small
        bool is_new_instance = tracking_data.getNumberOfInputPixels(input_id) >=
                               config_.min_new_instance_size;
        if (is_new_instance) {
          // Generate new globally unique InstanceID
          auto new_instance_id_iter =
              used_instance_ids_.emplace(&instance_id_manager_).first;
          // Update the instance id in the panoptic label
          input_to_output[input_id] = input_class_id * kPanopticLabelDivisor_ +
                                      static_cast<int>(*new_instance_id_iter);
          LOG_IF(INFO, config_.verbosity >= 1)
              << "Generated new instance id for class " << input_class_id;
        } else {
          input_to_output[input_id] = -1;
        }
      }
    }

    // Translate the id image - this will mostly affect thing classes
    for (auto it = input->idImagePtr()->begin<int>();
         it != input->idImagePtr()->end<int>(); ++it) {
      *it = input_to_output[*it];
    }

    // Publish Visualization if requested.
    if (visualizationIsOn()) {
      cv::Mat tracked_vis = renderer_.colorIdImage(input->idImage());
      visualize(input_vis, "input");
      visualize(rendered_vis_, "rendered");
      visualize(input->colorImage(), "color");
      visualize(tracked_vis, "tracked");
    }
  }
}

void SingleTSDFTracker::parseDetectronClasses(InputData* input) {
  std::unordered_map<int, int> detectron_to_class_id;
  for (auto it = input->idImagePtr()->begin<int>();
       it != input->idImagePtr()->end<int>(); ++it) {
    if (*it == 0) {
      // Zero indicates unknown class / no prediction.
      continue;
    }
    auto class_it = detectron_to_class_id.find(*it);
    if (class_it == detectron_to_class_id.end()) {
      // First time we encounter this ID, write to the map.
      const int class_id = input->detectronLabels().at(*it).category_id;
      detectron_to_class_id[*it] = class_id;
      *it = class_id;
    } else {
      *it = class_it->second;
    }
  }
}

void SingleTSDFTracker::parseDetectronPanopticLabels(InputData* input) {
  std::unordered_map<int, int> detectron_to_panoptic_ids;
  for (auto it = input->idImagePtr()->begin<int>();
       it != input->idImagePtr()->end<int>(); ++it) {
    if (*it == 0) {
      continue;
    }

    auto panoptic_id_it = detectron_to_panoptic_ids.find(*it);
    if (panoptic_id_it == detectron_to_panoptic_ids.end()) {
      const auto& label = input->detectronLabels().at(*it);
      int panoptic_id = label.category_id * kPanopticLabelDivisor_;
      if (label.is_thing) {
        // Instance ids should start from 1 so we reserve 0 for stuff classes
        panoptic_id += label.instance_id + 1;
      }
      detectron_to_panoptic_ids[*it] = panoptic_id;
      *it = panoptic_id;
    } else {
      *it = panoptic_id_it->second;
    }
  }
}

void SingleTSDFTracker::setup(SubmapCollection* submaps) {
  // Check if there is a loaded map.
  if (submaps->size() > 0) {
    Submap& map = *(submaps->begin());
    if (map.getConfig().voxel_size != config_.submap.voxel_size ||
        map.getConfig().voxels_per_side != config_.submap.voxels_per_side ||
        map.getConfig().truncation_distance !=
            config_.submap.truncation_distance ||
        map.getConfig().useClassLayer() != config_.submap.useClassLayer()) {
      LOG(WARNING)
          << "Loaded submap config does not match the specified config.";
    }
    map.setIsActive(true);
    map_id_ = map.getID();
  } else {
    // Allocate the single map.
    Submap* new_submap = submaps->createSubmap(config_.submap);
    new_submap->setLabel(PanopticLabel::kBackground);
    map_id_ = new_submap->getID();
  }
  submaps->setActiveFreeSpaceSubmapID(map_id_);
  is_setup_ = true;
}

TrackingInfoAggregator SingleTSDFTracker::computeTrackingData(
    SubmapCollection* submaps, InputData* input) {
  TrackingInfoAggregator tracking_data;

  // Process the input image
  tracking_data.insertInputImage(input->idImage(), input->depthImage(),
                                 globals_->camera()->getConfig(), 2);

  // Render the single TSDF map - one tracking info object for
  // each panoptic entity currently visible in the map
  auto tracking_infos = renderTrackingInfoVertices(*(submaps->begin()), *input);

  // Add tracking info to tracking data
  tracking_data.insertTrackingInfos(tracking_infos);

  // Render the data if required.
  if (visualizationIsOn()) {
    cv::Mat vis =
        cv::Mat::ones(globals_->camera()->getConfig().height,
                      globals_->camera()->getConfig().width, CV_32SC1) *
        -1;
    for (const TrackingInfo& info : tracking_infos) {
      for (const Eigen::Vector2i& point : info.getPoints()) {
        vis.at<int>(point.y(), point.x()) = info.getSubmapID();
      }
    }
    rendered_vis_ = renderer_.colorIdImage(vis);
  }

  return tracking_data;
}

std::vector<TrackingInfo> SingleTSDFTracker::renderTrackingInfoVertices(
    const Submap& submap, const InputData& input) {
  std::unordered_map<int, TrackingInfo> results_table;

  // Compute the maximum extent to lookup vertices.
  const Transformation T_C_S = input.T_M_C().inverse() * submap.getT_M_S();
  const Point origin_C = T_C_S * submap.getBoundingVolume().getCenter();
  const Camera::Config& cam_config = globals_->camera()->getConfig();
  std::vector<size_t> limits = {
      0u /* x_min */, static_cast<size_t>(cam_config.width) /* x_max */,
      0u /* y_min */, static_cast<size_t>(cam_config.height) /* y_max */};

  size_t subsampling_factor = 1;
  const Transformation T_S_C = T_C_S.inverse();
  const TsdfLayer& tsdf_layer = submap.getTsdfLayer();

  const float depth_tolerance =
      config_.depth_tolerance > 0
          ? config_.depth_tolerance
          : -config_.depth_tolerance * submap.getTsdfLayer().voxel_size();
  for (size_t u = limits[0]; u < limits[1];
       u += config_.rendering_subsampling) {
    for (size_t v = limits[2]; v < limits[3];
         v += config_.rendering_subsampling) {
      const float depth = input.depthImage().at<float>(v, u);
      if (depth < cam_config.min_range || depth > cam_config.max_range) {
        continue;
      }
      const cv::Vec3f& vertex = input.vertexMap().at<cv::Vec3f>(v, u);
      const Point P_S = T_S_C * Point(vertex[0], vertex[1], vertex[2]);
      const voxblox::BlockIndex block_index =
          tsdf_layer.computeBlockIndexFromCoordinates(P_S);
      const auto block_ptr = tsdf_layer.getBlockPtrByIndex(block_index);
      if (block_ptr) {
        const size_t voxel_index =
            block_ptr->computeLinearIndexFromCoordinates(P_S);
        const TsdfVoxel& voxel = block_ptr->getVoxelByLinearIndex(voxel_index);
        if (voxel.weight < 1e-6 || std::abs(voxel.distance) > depth_tolerance) {
          continue;
        }

        // Find or create a TrackingInfo for the panoptic entity of this voxel
        const auto& voxel_panoptic_id =
            submap
                .getClassLayer()  // FIXME: assert that map has the class layer?
                .getBlockConstPtrByIndex(block_index)
                ->getVoxelByLinearIndex(voxel_index)
                .getBelongingID();
        auto it = results_table.find(voxel_panoptic_id);
        if (it == results_table.end()) {
          it = results_table
                   .emplace(std::make_pair(voxel_panoptic_id,
                                           TrackingInfo(voxel_panoptic_id)))
                   .first;
        }
        TrackingInfo& result = it->second;

        result.insertVertexPoint(input.idImage().at<int>(v, u));
        if (visualizationIsOn()) {
          result.insertVertexVisualizationPoint(u, v);
        }
      }
    }
  }

  // Now move everything into an std::vector so we can directly pass
  // this to TrackingInfoAggregator::inserTrackingInfos
  std::vector<TrackingInfo> results_vector;
  for (auto& id_result_pair : results_table) {
    results_vector.push_back(std::move(id_result_pair.second));
  }
  return results_vector;
}

}  // namespace panoptic_mapping
