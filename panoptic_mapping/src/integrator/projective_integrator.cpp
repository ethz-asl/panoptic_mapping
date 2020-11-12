#include "panoptic_mapping/integrator/projective_integrator.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <voxblox/integrator/merge_integration.h>

namespace panoptic_mapping {

void ProjectiveIntegrator::Config::checkParams() const {
  checkParamGT(height, 0, "height");
  checkParamGT(width, 0, "width");
  checkParamGT(vx, 0.f, "vx");
  checkParamGT(vy, 0.f, "vy");
  checkParamGT(focal_length, 0.f, "focal_length");
  checkParamGT(integration_threads, 0, "integration_threads");
  checkParamCond(vx < width, "'vx' is required < 'width'");
  checkParamCond(vy < height, "'vy' is required < 'height'");
}

void ProjectiveIntegrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("width", &width);
  setupParam("height", &height);
  setupParam("vx", &vx);
  setupParam("vy", &vy);
  setupParam("focal_length", &focal_length);
  setupParam("max_range", &max_range);
  setupParam("min_range", &min_range);
  setupParam("use_weight_dropoff", &use_weight_dropoff);
  setupParam("use_constant_weight", &use_constant_weight);
  setupParam("foreign_rays_clear", &foreign_rays_clear);
  setupParam("sparsity_compensation_factor", &sparsity_compensation_factor);
  setupParam("max_weight", &max_weight);
  setupParam("interpolation_method", &interpolation_method);
  setupParam("integration_threads", &integration_threads);
}

ProjectiveIntegrator::ProjectiveIntegrator(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Setup the interpolator (one for each thread.)
  for (int i = 0; i < config_.integration_threads; ++i) {
    interpolators_.emplace_back(
        config_utilities::Factory::create<InterpolatorBase>(
            config_.interpolation_method));
  }

  // pre-compute the view frustum (top, right, bottom, left, plane normals)
  Eigen::Vector3f p1(-config_.vx, -config_.vy, config_.focal_length);
  Eigen::Vector3f p2(config_.width - config_.vx, -config_.vy,
                     config_.focal_length);
  Eigen::Vector3f normal = p1.cross(p2);
  view_frustum_.push_back(normal.normalized());
  p1 = Eigen::Vector3f(config_.width - config_.vx, config_.height - config_.vy,
                       config_.focal_length);
  normal = p2.cross(p1);
  view_frustum_.push_back(normal.normalized());
  p2 = Eigen::Vector3f(-config_.vx, config_.height - config_.vy,
                       config_.focal_length);
  normal = p1.cross(p2);
  view_frustum_.push_back(normal.normalized());
  p1 = Eigen::Vector3f(-config_.vx, -config_.vy, config_.focal_length);
  normal = p2.cross(p1);
  view_frustum_.push_back(normal.normalized());

  // allocate range image
  range_image_ = Eigen::MatrixXf(config_.height, config_.width);
}

ProjectiveIntegrator::ThreadSafeIndexGetter::ThreadSafeIndexGetter(
    std::vector<int> indices)
    : indices_(std::move(indices)), current_index_(0) {}

bool ProjectiveIntegrator::ThreadSafeIndexGetter::getNextIndex(int* index) {
  CHECK_NOTNULL(index);
  mutex_.lock();
  if (current_index_ >= indices_.size()) {
    mutex_.unlock();
    return false;
  }
  *index = indices_[current_index_];
  current_index_++;
  mutex_.unlock();
  return true;
}

void ProjectiveIntegrator::processImages(SubmapCollection* submaps,
                                         const Transformation& T_M_C,
                                         const cv::Mat& depth_image,
                                         const cv::Mat& color_image,
                                         const cv::Mat& id_image) {
  // Allocate all blocks in corresponding submaps.
  auto t1 = std::chrono::high_resolution_clock::now();
  allocateNewBlocks(submaps, T_M_C, depth_image, id_image);
  auto t2 = std::chrono::high_resolution_clock::now();

  // Find all active blocks that are in the field of view.
  // Note(schmluk): This could potentially also be included in the parallel part
  // but is already almost instantaneous.
  std::unordered_map<int, voxblox::BlockIndexList> block_lists;
  std::vector<int> id_list;
  for (auto& submap_ptr : *submaps) {
    if (!submap_ptr->isActive()) {
      continue;
    }
    if (!submapIsInViewFrustum(*submap_ptr, T_M_C)) {
      continue;
    }
    voxblox::BlockIndexList block_list;
    findVisibleBlocks(*submap_ptr, T_M_C, &block_list);
    if (!block_list.empty()) {
      id_list.emplace_back(submap_ptr->getID());
      block_lists[submap_ptr->getID()] = block_list;
    }
  }
  auto t3 = std::chrono::high_resolution_clock::now();

  // Integrate in parallel.
  ThreadSafeIndexGetter index_getter(id_list);
  std::vector<std::future<bool>> threads;
  for (int i = 0; i < config_.integration_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async, [this, &index_getter, &block_lists, submaps, &T_M_C,
                             &color_image, &id_image, i]() {
          int index;
          while (index_getter.getNextIndex(&index)) {
            this->updateSubmap(submaps->getSubmapPtr(index),
                               interpolators_[i].get(), block_lists[index],
                               T_M_C, color_image, id_image);
          }
          return true;
        }));
  }
  // Join all threads.
  for (auto& thread : threads) {
    thread.get();
  }
  auto t4 = std::chrono::high_resolution_clock::now();

  LOG_IF(INFO, config_.verbosity >= 3)
      << "Allocate: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
      << "ms, Find: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
      << "ms, Integrate: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
      << "ms.";
}

void ProjectiveIntegrator::updateSubmap(
    Submap* submap, InterpolatorBase* interpolator,
    const voxblox::BlockIndexList& block_indices, const Transformation& T_M_C,
    const cv::Mat& color_image, const cv::Mat& id_image) const {
  CHECK_NOTNULL(submap);
  // Update.
  for (const auto& index : block_indices) {
    updateTsdfBlock(submap, interpolator, index, T_M_C, color_image, id_image);
  }
}

void ProjectiveIntegrator::updateTsdfBlock(Submap* submap,
                                           InterpolatorBase* interpolator,
                                           const voxblox::BlockIndex& index,
                                           const Transformation& T_M_C,
                                           const cv::Mat& color_image,
                                           const cv::Mat& id_image) const {
  CHECK_NOTNULL(submap);
  // set up preliminaries
  if (!submap->getTsdfLayer().hasBlock(index)) {
    LOG(WARNING) << "Tried to access inexistent block '" << index.transpose()
                 << "' in submap " << submap->getID() << ".";
    return;
  }
  voxblox::Block<TsdfVoxel>& block =
      submap->getTsdfLayerPtr()->getBlockByIndex(index);
  block.updated().set();
  Transformation T_C_S =
      T_M_C.inverse() * submap->getT_M_S();  // p_C = T_C_M * T_M_S * p_S
  const float voxel_size = block.voxel_size();
  const float truncation_distance =
      voxel_size * 2.f;  // 2vs is used as an adaptive heuristic.
  const int id = submap->getID();

  // update all voxels
  for (size_t i = 0; i < block.num_voxels(); ++i) {
    voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
    const Point p_C = T_C_S * block.computeCoordinatesFromLinearIndex(
                                  i);  // voxel center in camera frame
    // Compute distance and weight.
    bool point_belongs_to_this_submap;
    float sdf;
    float weight;
    const bool is_free_space_submap =
        submap->getLabel() == PanopticLabel::kFreeSpace;
    if (!computeVoxelDistanceAndWeight(
            &sdf, &weight, &point_belongs_to_this_submap, interpolator, p_C,
            color_image, id_image, id, truncation_distance, voxel_size,
            is_free_space_submap)) {
      continue;
    }

    // Apply distance, color, and weight.
    if (point_belongs_to_this_submap || is_free_space_submap) {
      voxel.distance = (voxel.distance * voxel.weight +
                        std::max(std::min(truncation_distance, sdf),
                                 -1.f * truncation_distance) *
                            weight) /
                       (voxel.weight + weight);
      // only merge color near the surface and if point belongs to the submap.
      if (std::abs(sdf) < truncation_distance && point_belongs_to_this_submap) {
        voxel.color = Color::blendTwoColors(
            voxel.color, voxel.weight,
            interpolator->interpolateColor(color_image), weight);
      }
    } else {
      // Voxels that don't belong to the submap are 'cleared' if they are in
      // front of the surface.
      if (sdf > 0) {
        voxel.distance =
            (voxel.distance * voxel.weight + truncation_distance * weight) /
            (voxel.weight + weight);
      }
    }
    voxel.weight = std::min(voxel.weight + weight, config_.max_weight);
  }
}

bool ProjectiveIntegrator::computeVoxelDistanceAndWeight(
    float* sdf, float* weight, bool* point_belongs_to_this_submap,
    InterpolatorBase* interpolator, const Point& p_C,
    const cv::Mat& color_image, const cv::Mat& id_image, int submap_id,
    float truncation_distance, float voxel_size,
    bool is_free_space_submap) const {
  // Skip voxels that are too far or too close.
  if (p_C.z() < 0.0) {
    return false;
  }
  const float distance_to_voxel = p_C.norm();
  if (distance_to_voxel < config_.min_range ||
      distance_to_voxel > config_.max_range) {
    return false;
  }

  // Project the current voxel into the range image, only count points that fall
  // fully into the image.
  float u = p_C.x() * config_.focal_length / p_C.z() + config_.vx;
  if (std::ceil(u) >= config_.width || std::floor(u) < 0) {
    return false;
  }
  float v = p_C.y() * config_.focal_length / p_C.z() + config_.vy;
  if (std::ceil(v) >= config_.height || std::floor(v) < 0) {
    return false;
  }

  // Set up the interpolator and check whether this is a clearing or an updating
  // measurement.
  interpolator->computeWeights(u, v, submap_id, point_belongs_to_this_submap,
                               range_image_, id_image);
  if (!(*point_belongs_to_this_submap) &&
      !(config_.foreign_rays_clear || is_free_space_submap)) {
    return false;
  }

  //  Compute the signed distance.
  const float distance_to_surface =
      interpolator->interpolateDepth(range_image_);
  const float new_sdf = distance_to_surface - distance_to_voxel;
  if (new_sdf < -truncation_distance) {
    return false;
  }

  // Compute the weight of the measurement
  // Approximates the number of rays that would hit this voxel
  float observation_weight =
      std::pow(voxel_size * config_.focal_length / p_C.z(), 2.f);

  // weight reduction with distance squared (according to sensor noise models)
  if (!config_.use_constant_weight) {
    observation_weight /= std::pow(p_C.z(), 2.f);
  }

  // Apply weight drop-off if appropriate
  if (config_.use_weight_dropoff && *point_belongs_to_this_submap) {
    const voxblox::FloatingPoint dropoff_epsilon = voxel_size;
    if (new_sdf < -dropoff_epsilon) {
      observation_weight = observation_weight *
                           (truncation_distance + new_sdf) /
                           (truncation_distance - dropoff_epsilon);
      observation_weight = std::max(observation_weight, 0.f);
    }
  }

  // Apply sparsity compensation if appropriate
  if (config_.sparsity_compensation_factor != 1.f &&
      *point_belongs_to_this_submap) {
    if (std::abs(new_sdf) < truncation_distance) {
      observation_weight *= config_.sparsity_compensation_factor;
    }
  }

  // results (point belongs to this submap is already updated)
  *sdf = new_sdf;
  *weight = observation_weight;
  return true;
}

void ProjectiveIntegrator::findVisibleBlocks(
    const Submap& submap, const Transformation& T_M_C,
    voxblox::BlockIndexList* block_list) const {
  // setup
  voxblox::BlockIndexList all_blocks;
  submap.getTsdfLayer().getAllAllocatedBlocks(&all_blocks);
  const Transformation T_C_S =
      T_M_C.inverse() * submap.getT_M_S();  // p_C = T_C_M * T_M_S * p_S
  const FloatingPoint block_size = submap.getTsdfLayer().block_size();
  const FloatingPoint block_diag = std::sqrt(3.0f) * block_size / 2.0f;

  // iterate through all blocks
  for (auto& index : all_blocks) {
    auto& block = submap.getTsdfLayer().getBlockByIndex(index);
    const Point p_C =
        T_C_S * (block.origin() + Point(1, 1, 1) * block_size /
                                      2.0);  // center point of the block

    if (blockIsInViewFrustum(p_C, block_diag)) {
      block_list->push_back(index);
    }
  }
}

bool ProjectiveIntegrator::submapIsInViewFrustum(
    const Submap& submap, const Transformation& T_M_C) const {
  // Assumes the range image is read to use the cached max_range_in_image_.
  // Sort out points that don't meet the criteria of
  // 1: in front, 2: close, 3: in view frustum.
  const FloatingPoint radius = submap.getBoundingVolume().getRadius();
  const Point center_C = T_M_C.inverse() * submap.getT_M_S() *
                         submap.getBoundingVolume().getCenter();
  if (center_C.z() < -radius) {
    return false;
  }
  if (center_C.norm() > max_range_in_image_ + radius) {
    return false;
  }
  for (const Point& view_frustum_plane : view_frustum_) {
    if (center_C.dot(view_frustum_plane) < -radius) {
      return false;
    }
  }
  return true;
}

bool ProjectiveIntegrator::blockIsInViewFrustum(const Point& center_point_C,
                                                float block_diag_half) const {
  // Assumes the range image is read to use the cached max_range_in_image_.
  // Sort out points that don't meet the criteria of
  // 1: in front, 2: close, 3: in view frustum.
  if (center_point_C.z() < -block_diag_half) {
    return false;
  }
  if (center_point_C.norm() > max_range_in_image_ + block_diag_half) {
    return false;
  }
  for (const Point& view_frustum_plane : view_frustum_) {
    if (center_point_C.dot(view_frustum_plane) < -block_diag_half) {
      return false;
    }
  }
  return true;
}

void ProjectiveIntegrator::allocateNewBlocks(SubmapCollection* submaps,
                                             const Transformation& T_M_C,
                                             const cv::Mat& depth_image,
                                             const cv::Mat& id_image) {
  // This method also resets the depth image.
  range_image_.setZero();
  max_range_in_image_ = 0.f;

  // Parse through each point to allocate instance + background blocks.
  std::unordered_set<Submap*> touched_submaps;
  for (int v = 0; v < depth_image.rows; v++) {
    for (int u = 0; u < depth_image.cols; u++) {
      float z = depth_image.at<float>(v, u);
      float x = (static_cast<float>(u) - config_.vx) * z / config_.focal_length;
      float y = (static_cast<float>(v) - config_.vy) * z / config_.focal_length;
      float ray_distance = std::sqrt(x * x + y * y + z * z);
      range_image_(v, u) = ray_distance;
      if (ray_distance > config_.max_range) {
        continue;
      }
      max_range_in_image_ = std::max(max_range_in_image_, ray_distance);
      Submap* submap =
          submaps->getSubmapPtr(static_cast<int>(id_image.at<uchar>(v, u)));
      const Point p_S = submap->getT_S_M() * T_M_C *
                        Point(x, y, z);  // p_S = T_S_M * T_M_C * p_C
      submap->getTsdfLayerPtr()->allocateBlockPtrByCoordinates(p_S);
      touched_submaps.insert(submap);
    }
  }

  // Allocate all potential free space blocks.
  Submap* space = submaps->getSubmapPtr(submaps->getActiveFreeSpaceSubmapID());
  const float block_size = space->getTsdfLayer().block_size();
  const float block_diag = std::sqrt(3.f) * block_size / 2.f;
  const Transformation T_C_S = T_M_C.inverse() * space->getT_M_S();
  const Point camera_S = T_C_S.inverse().getPosition();  // T_S_C
  const int max_steps = std::floor((max_range_in_image_ + block_diag) /
                                   space->getTsdfLayer().block_size());
  for (int x = -max_steps; x <= max_steps; ++x) {
    for (int y = -max_steps; y <= max_steps; ++y) {
      for (int z = -max_steps; z <= max_steps; ++z) {
        const Point offset(x, y, z);
        const Point candidate_S = camera_S + offset * block_size;
        if (blockIsInViewFrustum(T_C_S * candidate_S, block_diag)) {
          space->getTsdfLayerPtr()->allocateBlockPtrByCoordinates(candidate_S);
        }
      }
    }
  }

  // Update all bounding volumes. This is currently done in every integration
  // step since it's not too expensive and won't do anything if no new block
  // was allocated.
  for (auto& submap : touched_submaps) {
    submap->getBoundingVolumePtr()->update();
  }
  space->getBoundingVolumePtr()->update();
}

}  // namespace panoptic_mapping
