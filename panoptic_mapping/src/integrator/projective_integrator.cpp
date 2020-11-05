#include "panoptic_mapping/integrator/projective_integrator.h"

#include <voxblox/integrator/merge_integration.h>

#include <algorithm>
#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

namespace panoptic_mapping {

ProjectiveIntegrator::Config ProjectiveIntegrator::Config::isValid() const {
  Config config(*this);
  // setup default values
  if (vx <= 0) {
    config.vx = width / 2.0;
  }
  if (vy <= 0) {
    config.vy = config.height / 2.0;
  }
  if (integration_threads <= 0) {
    config.integration_threads = std::thread::hardware_concurrency();
  }

  // check params are valid
  CHECK_GT(height, 0) << "'height' must be a positive integer.";
  CHECK_GT(width, 0) << "'width' must be a positive integer.";
  CHECK_GT(config.vx, 0) << "'vx' must be a positive float.";
  CHECK_GT(config.vy, 0) << "'vy' must be a positive float.";
  CHECK_GT(focal_length, 0) << "'focal_length' must be a positive integer.";
  CHECK_GT(config.integration_threads, 0)
      << "'integration_threads' must be a positive integer.";
  CHECK_LT(config.vx, width) << "'vx' must be smaller than 'width'.";
  CHECK_LT(config.vy, height) << "'vy' must be smaller than 'height'.";

  return config;
}

ProjectiveIntegrator::ProjectiveIntegrator(const Config& config)
    : config_(config.isValid()) {
  // setup the interpolator
  interpolator_ = InterpolatorFactory::create(config_.interpolation_method);

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

void ProjectiveIntegrator::processImages(SubmapCollection* submaps,
                                         const Transformation& T_M_C,
                                         const cv::Mat& depth_image,
                                         const cv::Mat& color_image,
                                         const cv::Mat& id_image) {
  // allocate all blocks in corresponding submaps
  auto t1 = std::chrono::high_resolution_clock::now();
  allocateNewBlocks(submaps, T_M_C, depth_image, id_image);
  auto t2 = std::chrono::high_resolution_clock::now();

  // find all active blocks that are in the field of view
  std::vector<voxblox::BlockIndexList> block_lists;
  std::vector<int> id_list;
  for (auto& submap_ptr : *submaps) {
    if (!submap_ptr->isActive()) {
      continue;
    }
    voxblox::BlockIndexList block_list;
    findVisibleBlocks(*submap_ptr, T_M_C, &block_list);
    if (!block_list.empty()) {
      block_lists.push_back(block_list);
      id_list.push_back(submap_ptr->getID());
    }
  }
  auto t3 = std::chrono::high_resolution_clock::now();

  // integrate
  for (size_t i = 0; i < block_lists.size(); ++i) {
    updateSubmap(block_lists[i], submaps->getSubmapPtr(id_list[i]), T_M_C,
                 color_image, id_image);
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
    const voxblox::BlockIndexList& block_indices, Submap* submap,
    const Transformation& T_M_C, const cv::Mat& color_image,
    const cv::Mat& id_image) {
  CHECK_NOTNULL(submap);
  // update
  for (const auto& index : block_indices) {
    updateTsdfBlock(index, submap, T_M_C, color_image, id_image);
  }
}

void ProjectiveIntegrator::updateTsdfBlock(const voxblox::BlockIndex& index,
                                           Submap* submap,
                                           const Transformation& T_M_C,
                                           const cv::Mat& color_image,
                                           const cv::Mat& id_image) {
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
      voxel_size * 2.f;  // 2vs is used as an adaptive heuristic
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
    if (!computeVoxelDistanceAndWeight(
            &sdf, &weight, &point_belongs_to_this_submap, p_C, color_image,
            id_image, id, truncation_distance, voxel_size)) {
      continue;
    }

    // Apply distance, color, and weight.
    if (point_belongs_to_this_submap) {
      voxel.distance = (voxel.distance * voxel.weight +
                        std::min(truncation_distance, sdf) * weight) /
                       (voxel.weight + weight);
      // only merge color near the surface and if point belongs to the submap.
      if (std::abs(sdf) < truncation_distance) {
        voxel.color = Color::blendTwoColors(
            voxel.color, voxel.weight,
            interpolator_->interpolateColor(color_image), weight);
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
    const Point& p_C, const cv::Mat& color_image, const cv::Mat& id_image,
    int submap_id, float truncation_distance, float voxel_size) {
  // skip voxels that are too far or too close
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
  interpolator_->computeWeights(u, v, submap_id, point_belongs_to_this_submap,
                                range_image_, id_image);
  if (!config_.foreign_rays_clear && !point_belongs_to_this_submap) {
    return false;
  }

  //  compute the signed distance
  const float distance_to_surface =
      interpolator_->interpolateDepth(range_image_);
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
  if (config_.use_weight_dropoff && point_belongs_to_this_submap) {
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
      point_belongs_to_this_submap) {
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
    voxblox::BlockIndexList* block_list) {
  // setup
  voxblox::BlockIndexList all_blocks;
  submap.getTsdfLayer().getAllAllocatedBlocks(&all_blocks);
  Transformation T_C_S =
      T_M_C.inverse() * submap.getT_M_S();  // p_C = T_C_M * T_M_S * p_S
  float block_size = submap.getTsdfLayer().block_size();
  float block_diag = std::sqrt(3.0f) * block_size / 2.0f;

  // iterate through all blocks
  for (auto& index : all_blocks) {
    auto& block = submap.getTsdfLayer().getBlockByIndex(index);
    const Point p_C =
        T_C_S * (block.origin() + Point(1, 1, 1) * block_size /
                                      2.0);  // center point of the block

    // sort out points that don't meet the criteria of
    // 1: in front, 2: close, 3: in view frustum.
    if (p_C.z() < block_size / 2.0) {
      continue;
    }
    if (p_C.norm() > max_range_in_image_ + block_diag) {
      continue;
    }
    if (p_C.dot(view_frustum_[0]) < -block_diag) {
      continue;
    }
    if (p_C.dot(view_frustum_[1]) < -block_diag) {
      continue;
    }
    if (p_C.dot(view_frustum_[2]) < -block_diag) {
      continue;
    }
    if (p_C.dot(view_frustum_[3]) < -block_diag) {
      continue;
    }
    block_list->push_back(index);
  }
}

void ProjectiveIntegrator::allocateNewBlocks(SubmapCollection* submaps,
                                             const Transformation& T_M_C,
                                             const cv::Mat& depth_image,
                                             const cv::Mat& id_image) {
  // This method also resets the depth image
  range_image_.setZero();
  max_range_in_image_ = 0.f;

  // parse through each point
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
    }
  }
}

}  // namespace panoptic_mapping
