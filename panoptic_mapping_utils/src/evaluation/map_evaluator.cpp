#include "panoptic_mapping_utils/evaluation/map_evaluator.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <pcl/io/ply_io.h>
#include <ros/ros.h>
#include <voxblox/interpolator/interpolator.h>

#include "panoptic_mapping_utils/evaluation/progress_bar.h"

namespace panoptic_mapping {

void MapEvaluator::EvaluationRequest::checkParams() const {
  checkParamGT(maximum_distance, 0.f, "maximum_distance");
}

void MapEvaluator::EvaluationRequest::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("map_file", &map_file);
  setupParam("ground_truth_pointcloud_file", &ground_truth_pointcloud_file);
  setupParam("maximum_distance", &maximum_distance);
  setupParam("evaluate", &evaluate);
  setupParam("visualize", &visualize);
  setupParam("compute_coloring", &compute_coloring);
  setupParam("color_by_max_error", &color_by_max_error);
  setupParam("ignore_truncated_points", &ignore_truncated_points);
}

MapEvaluator::MapEvaluator(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  auto config =
      config_utilities::getConfigFromRos<SubmapVisualizer::Config>(nh_private_);
  visualizer_ = std::make_unique<SubmapVisualizer>(config, nullptr);
}

bool MapEvaluator::evaluate(const EvaluationRequest& request) {
  if (!request.isValid(true)) {
    return false;
  }
  LOG_IF(INFO, request.verbosity >= 2) << "Processing: \n"
                                       << request.toString();

  // Load the groundtruth pointcloud.
  if (request.evaluate || request.compute_coloring) {
    if (!request.ground_truth_pointcloud_file.empty()) {
      gt_ptcloud_ = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
      if (pcl::io::loadPLYFile<pcl::PointXYZ>(
              request.ground_truth_pointcloud_file, *gt_ptcloud_) != 0) {
        LOG(ERROR) << "Could not load ground truth point cloud from '"
                   << request.ground_truth_pointcloud_file << "'.";
        gt_ptcloud_.reset();
        return false;
      }
      LOG_IF(INFO, request.verbosity >= 2) << "Loaded ground truth pointcloud";
    }
    if (!gt_ptcloud_) {
      LOG(ERROR) << "No ground truth pointcloud loaded.";
      return false;
    }
  }

  // Load the map to evaluate.
  if (!request.map_file.empty()) {
    // Load the map.
    const std::string extension =
        request.map_file.substr(request.map_file.find_last_of('.'));
    size_t separator = request.map_file.find_last_of('/');
    target_directory_ = request.map_file.substr(0, separator);
    target_map_name_ = request.map_file.substr(
        separator + 1,
        request.map_file.length() - separator - extension.length() - 1);

    if (extension == ".panmap") {
      // Load panoptic map.
      use_voxblox_ = false;
      submaps_ = std::make_shared<SubmapCollection>();
      if (!submaps_->loadFromFile(request.map_file)) {
        LOG(ERROR) << "Could not load panoptic map from '" << request.map_file
                   << "'.";
        submaps_.reset();
        return false;
      }
      planning_ = std::make_unique<PlanningInterface>(submaps_);
      LOG_IF(INFO, request.verbosity >= 2) << "Loaded the target panoptic map.";
    } else if (extension == ".vxblx") {
      use_voxblox_ = true;
      voxblox::io::LoadLayer<voxblox::TsdfVoxel>(request.map_file, &voxblox_);
      LOG_IF(INFO, request.verbosity >= 2) << "Loaded the target voxblox map.";
    } else {
      LOG(ERROR) << "cannot load file of unknown extension '"
                 << request.map_file << "'.";
      return false;
    }
  }
  if (!submaps_ && !use_voxblox_) {
    LOG(ERROR) << "No panoptic map loaded.";
    return false;
  }

  // Setup output file
  if (request.evaluate) {
    std::string out_file_name =
        target_directory_ + "/" + target_map_name_ + "_evaluation_data.csv";
    output_file_.open(out_file_name, std::ios::out);
    if (!output_file_.is_open()) {
      LOG(ERROR) << "Failed to open output file '" << out_file_name << "'.";
      return false;
    }

    // Evaluate.
    LOG_IF(INFO, request.verbosity >= 2) << "Computing reconstruction error:";
    computeReconstructionError(request);
    output_file_.close();
  }

  // Compute visualization if required.
  if (request.compute_coloring) {
    LOG_IF(INFO, request.verbosity >= 2) << "Computing visualization coloring:";
    visualizeReconstructionError(request);
  }

  // Display the mesh.
  if (request.visualize) {
    LOG_IF(INFO, request.verbosity >= 2) << "Publishing mesh.";
    publishVisualization();
  }

  LOG_IF(INFO, request.verbosity >= 2) << "Done.";
  return true;
}

void MapEvaluator::computeReconstructionError(
    const EvaluationRequest& request) {
  // Go through each point, use trilateral interpolation to figure out the
  // distance at that point.

  // Setup.
  uint64_t total_points = 0;
  uint64_t unknown_points = 0;
  uint64_t truncated_points = 0;
  std::vector<float> abserror;
  abserror.reserve(gt_ptcloud_->size());  // Just reserve the worst case.

  // TEST
  //  std::vector<int> inactive_ids;
  //  for (auto& s : *submaps_) {
  //    s->setIsActive(s->getInstanceID() == 0);
  //    if (!s->isActive()) {
  //      inactive_ids.push_back(s->getID());
  //    }
  //  }
  //  for (int id : inactive_ids) {
  //    submaps_->removeSubmap(id);
  //  }

  // Setup progress bar.
  const uint64_t interval = gt_ptcloud_->size() / 100;
  uint64_t count = 0;
  ProgressBar bar;

  // Evaluate gt pcl based(# gt points within < trunc_dist)
  std::unique_ptr<voxblox::Interpolator<voxblox::TsdfVoxel>> interp;

  if (use_voxblox_) {
    interp.reset(new voxblox::Interpolator<voxblox::TsdfVoxel>(voxblox_.get()));
  }

  for (const auto& pcl_point : *gt_ptcloud_) {
    const Point point(pcl_point.x, pcl_point.y, pcl_point.z);
    total_points++;

    float distance;
    if (use_voxblox_) {
      if (interp->getDistance(point, &distance, true)) {
        if (std::abs(distance) > request.maximum_distance) {
          truncated_points++;
          if (!request.ignore_truncated_points) {
            abserror.push_back(request.maximum_distance);
          }
        } else {
          abserror.push_back(std::abs(distance));
        }
      } else {
        unknown_points++;
      }
    } else {
      if (planning_->getDistance(point, &distance, false, false)) {
        if (std::abs(distance) > request.maximum_distance) {
          truncated_points++;
          if (!request.ignore_truncated_points) {
            abserror.push_back(request.maximum_distance);
          }
        } else {
          abserror.push_back(std::abs(distance));
        }
      } else {
        unknown_points++;
      }
    }

    // Progress bar.
    if (count % interval == 0) {
      bar.display(static_cast<float>(count) / gt_ptcloud_->size());
    }
    count++;
  }
  bar.display(1.f);

  // Report summary.
  float mean = 0.0;
  float rmse = 0.0;
  for (auto value : abserror) {
    mean += value;
    rmse += std::pow(value, 2);
  }
  if (!abserror.empty()) {
    mean /= static_cast<float>(abserror.size());
    rmse = std::sqrt(rmse / static_cast<float>(abserror.size()));
  }
  float stddev = 0.0;
  for (auto value : abserror) {
    stddev += std::pow(value - mean, 2.0);
  }
  if (abserror.size() > 2) {
    stddev = sqrt(stddev / static_cast<float>(abserror.size() - 1));
  }

  output_file_ << "MeanError [m],StdError [m],RMSE [m],TotalPoints [1],"
               << "UnknownPoints [1],TruncatedPoints [1]\n";
  output_file_ << mean << "," << stddev << "," << rmse << "," << total_points
               << "," << unknown_points << "," << truncated_points << "\n";
}

void MapEvaluator::computeErrorHistogram(const EvaluationRequest& request) {
  // create histogram of error distribution
  //    std::vector<int> histogram(hist_bins_);
  //    float bin_size = truncation_distance / ((float) hist_bins_ - 1.0);
  //    for (int i = 0; i < abserror.size(); ++i) {
  //      int bin = (int) floor(abserror[i] / bin_size);
  //      if (bin < 0 || bin >= hist_bins_) {
  //        std::cout << "Bin Error at bin " << bin << ", value " << abserror[i]
  //                  << std::endl;
  //        continue;
  //      }
  //      histogram[bin] += 1;
  //    }
  //    hist_file_ << map_name;
  //    for (int i = 0; i < histogram.size(); ++i) {
  //      hist_file_ << "," << histogram[i];
  //    }
  //    hist_file_ << "\n";
}

void MapEvaluator::visualizeReconstructionError(
    const EvaluationRequest& request) {
  // Coloring: grey -> unknown, green -> 0 error, red -> maximum error,
  // purple -> truncated to max error.

  constexpr int max_number_of_neighbors_factor = 25000;  // points per cubic
  // metre depending on voxel size for faster nn search.

  // Build a Kd tree for point lookup.
  TreeData kdtree_data;
  kdtree_data.points.reserve(gt_ptcloud_->size());
  for (const auto& point : *gt_ptcloud_) {
    kdtree_data.points.emplace_back(point.x, point.y, point.z);
  }
  KDTree kdtree(3, kdtree_data, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree.buildIndex();

  // Setup progress bar.
  float counter = 0.f;
  float max_counter = 0.f;
  ProgressBar bar;
  for (auto& submap : *submaps_) {
    voxblox::BlockIndexList block_list;
    submap.getTsdfLayer().getAllAllocatedBlocks(&block_list);
    max_counter += block_list.size();
  }

  // Parse all submaps
  for (auto& submap : *submaps_) {
    if (submap.getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }
    const size_t num_voxels_per_block =
        std::pow(submap.getTsdfLayer().voxels_per_side(), 3);
    const float voxel_size = submap.getTsdfLayer().voxel_size();
    const float voxel_size_sqr = voxel_size * voxel_size;
    const float truncation_distance = submap.getConfig().truncation_distance;
    const int max_number_of_neighbors =
        max_number_of_neighbors_factor / std::pow(1.f / voxel_size, 2.f);
    voxblox::Interpolator<TsdfVoxel> interpolator(
        submap.getTsdfLayerPtr().get());

    // Parse all voxels.
    voxblox::BlockIndexList block_list;
    submap.getTsdfLayer().getAllAllocatedBlocks(&block_list);
    int block = 0;
    for (auto& block_index : block_list) {
      voxblox::Block<TsdfVoxel>& block =
          submap.getTsdfLayerPtr()->getBlockByIndex(block_index);
      for (size_t linear_index = 0; linear_index < num_voxels_per_block;
           ++linear_index) {
        TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
        if (voxel.distance > truncation_distance ||
            voxel.distance < -truncation_distance) {
          continue;  // these voxels can never be surface.
        }
        Point center = block.computeCoordinatesFromLinearIndex(linear_index);

        // Find surface points within 1 voxel size.
        // Note(schmluk): Use N neighbor search wih increasing N since radius
        // search is ridiculously slow.
        float query_pt[3] = {center.x(), center.y(), center.z()};
        std::vector<size_t> ret_index(max_number_of_neighbors);
        std::vector<float> out_dist_sqr(max_number_of_neighbors);
        int num_results =
            kdtree.knnSearch(&query_pt[0], max_number_of_neighbors,
                             &ret_index[0], &out_dist_sqr[0]);

        if (num_results == 0) {
          // No nearby surface.
          voxel.color = Color(128, 128, 128);
          continue;
        }

        // Get average error.
        float total_error = 0.f;
        float max_error = 0.f;
        int counted_voxels = 0;
        float min_dist_sqr = 1000.f;
        for (int i = 0; i < num_results; ++i) {
          min_dist_sqr = std::min(min_dist_sqr, out_dist_sqr[i]);
          if (out_dist_sqr[i] > voxel_size_sqr) {
            continue;
          }
          voxblox::FloatingPoint distance;
          if (interpolator.getDistance(kdtree_data.points[ret_index[i]],
                                       &distance, true)) {
            const float error = std::abs(distance);
            total_error += error;
            max_error = std::max(max_error, error);
            counted_voxels++;
          }
        }
        // Coloring.
        if (counted_voxels == 0) {
          counted_voxels = 1;
          total_error += std::sqrt(min_dist_sqr);
        }
        float frac;
        if (request.color_by_max_error) {
          frac = std::min(max_error, request.maximum_distance) /
                 request.maximum_distance;
        } else {
          frac =
              std::min(total_error / counted_voxels, request.maximum_distance) /
              request.maximum_distance;
        }

        float r = std::min((frac - 0.5f) * 2.f + 1.f, 1.f) * 255.f;
        float g = (1.f - frac) * 2.f * 255.f;
        if (frac <= 0.5f) {
          g = 190.f + 130.f * frac;
        }
        voxel.color = voxblox::Color(r, g, 0);
      }

      // Show progress.
      counter += 1.f;
      bar.display(counter / max_counter);
    }
    submap.updateMesh(false);
  }

  // Store colored submaps.
  std::string output_name =
      target_directory_ + "/" + target_map_name_ + "_evaluated_" +
      (request.color_by_max_error ? "max" : "mean") + ".panmap";
  submaps_->saveToFile(output_name);
}

void MapEvaluator::publishVisualization() {
  // Make sure the tfs arrive otherwise the mesh will be discarded.
  visualizer_->reset();
  visualizer_->visualizeAll(submaps_.get());
}

}  // namespace panoptic_mapping
