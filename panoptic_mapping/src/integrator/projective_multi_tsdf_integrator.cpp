#include "panoptic_mapping/integrator/projective_multi_tsdf_integrator.h"

#include <algorithm>
#include <iostream>
#include <list>
#include <vector>
#include <thread>

#include <voxblox/integrator/integrator_utils.h>

namespace panoptic_mapping {

constexpr float kFloatEpsilon = 1e-6;    // Used for weights

void ProjectiveMutliTSDFIntegrator::setupFromConfig(IntegratorBase::Config *config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config *>(config);
  if (cfg) {
    config_ = *cfg;
  } else {
    LOG(ERROR) << "Failed to setup: config is not of type 'ProjectiveMutliTSDFIntegrator::Config'.";
    return;
  }
  // check the config for validity
  CHECK_GT(config_.sensor_horizontal_resolution, 0)
    << "The horizontal sensor resolution must be a positive integer.";
  CHECK_GT(config_.sensor_vertical_resolution, 0)
    << "The vertical sensor resolution must be a positive integer.";
  CHECK_GT(config_.sensor_vertical_fov_deg, 0.0)
    << "The vertical field of view of the sensor must be a positive float.";

  // constants
  c_vertical_fov_rad_ = config_.sensor_vertical_fov_deg * M_PI / 180.0;
  c_focal_length_ =
      (float) config_.sensor_vertical_resolution / 2.0 * std::tan(config_.sensor_vertical_fov_deg / 90.0 * M_PI);

  // initialize data memory
  range_image_ = Eigen::MatrixXf(config_.sensor_vertical_resolution, config_.sensor_horizontal_resolution);
  id_image_ = Eigen::MatrixXi(config_.sensor_vertical_resolution, config_.sensor_horizontal_resolution);
}

void ProjectiveMutliTSDFIntegrator::processPointcloud(SubmapCollection *submaps,
                                                      const Transformation &T_M_C,
                                                      const Pointcloud &pointcloud,
                                                      const Colors &colors,
                                                      const std::vector<int> &ids) {
  CHECK_NOTNULL(submaps);
  CHECK_EQ(pointcloud.size(), colors.size());
  CHECK_EQ(pointcloud.size(), ids.size());

  // convert and read the pointcloud
  readPointcloud(T_M_C, pointcloud, colors, ids);

  // integrate all submaps
  for (auto &submap : *submaps) {
    integratePointcloudToSubmaps(&submap);
  }
  std::cout << "Integrated points into " << submaps->size() << " submaps." << std::endl;
}

void ProjectiveMutliTSDFIntegrator::readPointcloud(const Transformation &T_M_C,
                                                   const Pointcloud &pointcloud,
                                                   const Colors &colors,
                                                   const std::vector<int> &ids) {
  // reset previous measurements
  range_image_.setZero();
  id_image_.setZero();
  bearing_points_.clear();
  distances_.clear();
  ids_.clear();
  bearing_points_.reserve(pointcloud.size()); // max number of points so allocate once
  distances_.reserve(pointcloud.size());
  ids_.reserve(pointcloud.size());

  // cache temporary data
  T_M_C_ = T_M_C;

  int t = 0;
  int t2 = 0;
  int t3 = 0;
  int t4 = 0;
  //voxblox::Point t_G_C_scaled = T_G_C.getPosition() * layer_->block_size_inv();
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    const Point &point_C = pointcloud[i];
    // Compute the point's bearing vector
    float distance = point_C.norm();
    if (std::abs(distance) < kFloatEpsilon) {
      // Avoid divisions by zero
      t2++;
      continue;
    }
    Point point_C_bearing = point_C / distance;

    // Get the bearing vector's coordinates in the range image
    // NOTE: In this first implementation we only update the range image at its
    //       nearest neighbor coordinate
    int h, w;
    if (!bearingToImage(point_C_bearing, &h, &w)) {
      t3++;
      continue;
    }

    // store bearing and distance for different integration steps
    if (range_image_(h, w) < kFloatEpsilon) {
      bearing_points_.push_back(point_C_bearing);
      distances_.push_back(distance);
      ids_.push_back(ids[i]);
    }

    // Set the range image distance
    if (range_image_(h, w) < kFloatEpsilon || range_image_(h, w) > distance) {
      // If the range pixel has already been updated by another ray,
      // we resolve the conflict by taking the minimum distance
      range_image_(h, w) = distance;
      id_image_(h, w) = ids[i];
      t4++;
    }
    t++;
  }
  std::cout << t << "/" << pointcloud.size() << " points read into range image (" << t2 << " too close," << t3
            << " failed bearing," << t4 << " overwritten)." << std::endl;
}

void ProjectiveMutliTSDFIntegrator::integratePointcloudToSubmaps(Submap *submap) {
  CHECK_NOTNULL(submap);
  // transformation of submap to camera
  Transformation T_S_C = T_M_C_; //submap->T_M_S.inv() *T_M_C_

  // set the integration layer
  setTsdfIntegrationLayer(submap->getTsdfLayerPtr());

  // find touched blocks
  voxblox::IndexSet touched_block_indices, containing_block_indices;
  findTouchedBlocks(T_S_C, submap->getID(), &touched_block_indices, &containing_block_indices);

  //std::cout <<"Subamp " << submap->getID() <<": Touched " << touched_block_indices.size() << "/" << containing_block_indices.size() << " blocks." << std::endl;

  // Allocate all the blocks
  // NOTE: The layer's BlockHashMap is not thread safe, so doing it here before
  //       we start the multi-threading makes life easier
  for (const auto &block_index : containing_block_indices) {
    layer_->allocateBlockPtrByIndex(block_index);
  }

  // Process all blocks
  if (config_.integrator_threads == 1) {
    updateTsdfBlocks(T_S_C, touched_block_indices);
  } else {
    std::vector<voxblox::IndexSet> block_index_subsets(config_.integrator_threads);
    size_t idx = 0;
    for (const auto &block_index : touched_block_indices) {
      block_index_subsets[idx % config_.integrator_threads].emplace(block_index);
      idx++;
    }
    std::list<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &ProjectiveMutliTSDFIntegrator::updateTsdfBlocks, this, T_S_C, block_index_subsets[i]);
    }
    for (std::thread &integration_thread : integration_threads) {
      integration_thread.join();
    }
  }

}

void ProjectiveMutliTSDFIntegrator::setTsdfIntegrationLayer(voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer) {
  CHECK_NOTNULL(tsdf_layer);
  layer_ = tsdf_layer;
  voxel_size_ = layer_->voxel_size();
  truncation_distance_ = 2.0 * voxel_size_;  // heuristic for the moment
}

void ProjectiveMutliTSDFIntegrator::findTouchedBlocks(const Transformation &T_S_C,
                                                      int id,
                                                      voxblox::IndexSet *touched_block_indices,
                                                      voxblox::IndexSet *containing_blocks_indices) {
  CHECK_NOTNULL(touched_block_indices);
  CHECK_NOTNULL(containing_blocks_indices);

  // This is the implementation taken from projective_tsdf_integrator.
  voxblox::Point t_G_C_scaled = T_S_C.getPosition() * layer_->block_size_inv();
  for (size_t i = 0; i < bearing_points_.size(); ++i) {
    // Mark the blocks hit by this ray
    if (config_.min_ray_length_m <= distances_[i] &&
        distances_[i] <= config_.max_ray_length_m) {
      Point point_G_scaled = T_S_C *
          (bearing_points_[i] * (distances_[i] + truncation_distance_)) * layer_->block_size_inv();
      Point t_G_C_truncated_scaled;
      if (config_.voxel_carving_enabled) {
        t_G_C_truncated_scaled = t_G_C_scaled;
      } else {
        t_G_C_truncated_scaled =
            point_G_scaled - T_S_C * bearing_points_[i] * truncation_distance_ * layer_->block_size_inv();
      }
      voxblox::GlobalIndex block_index;
      voxblox::RayCaster ray_caster(point_G_scaled, t_G_C_truncated_scaled);
      while (ray_caster.nextRayIndex(&block_index)) {
        touched_block_indices->insert(block_index.cast<voxblox::IndexElement>());
      }
      // This lists all blocks which contain a point (last step of the ray ends in this block)
      if (ids_[i] == id) {
        containing_blocks_indices->insert(block_index.cast<voxblox::IndexElement>());
      }
    }
  }
}

void ProjectiveMutliTSDFIntegrator::updateTsdfBlocks(const Transformation &T_G_C,
                                                     const voxblox::IndexSet &touched_block_indices) {
  for (const voxblox::BlockIndex &block_index : touched_block_indices) {
    if (!layer_->hasBlock(block_index)) {
      continue;
    }
    voxblox::Block<TsdfVoxel>::Ptr block_ptr =
        layer_->getBlockPtrByIndex(block_index);
    block_ptr->updated().set();

    for (size_t linear_index = 0u; linear_index < num_voxels_per_block_; ++linear_index) {
      TsdfVoxel *tsdf_voxel = &block_ptr->getVoxelByLinearIndex(linear_index);
      const Point t_C_voxel = T_G_C.inverse() * block_ptr->computeCoordinatesFromLinearIndex(linear_index);
      updateTsdfVoxel(T_G_C, t_C_voxel, tsdf_voxel);
    }
  }
}

void ProjectiveMutliTSDFIntegrator::updateTsdfVoxel(const Transformation &T_G_C,
                                                    const Point &t_C_voxel,
                                                    TsdfVoxel *tsdf_voxel) {
  // Skip voxels that are too far or too close
  const float distance_to_voxel = t_C_voxel.norm();
  if (distance_to_voxel < config_.min_ray_length_m ||
      distance_to_voxel > config_.max_ray_length_m) {
    return;
  }

  // Project the current voxel into the range image
  float h, w;
  if (!bearingToImage(t_C_voxel / distance_to_voxel, &h, &w)) {
    return;
  }

  // Compute the signed distance
  const float distance_to_surface = interpolate(h, w);
  const float sdf = distance_to_surface - distance_to_voxel;

  // Approximate how many rays would have updated the voxel
  // NOTE: We do this to reflect that we are more certain about
  //       the updates applied to nearer voxels
  const float
      num_rays_intersecting_voxel = ray_intersections_per_distance_squared_ / (distance_to_voxel * distance_to_voxel);

  // Skip voxels that fall outside the TSDF truncation distance
  if (sdf < -truncation_distance_) {
    return;
  }
  if (!config_.voxel_carving_enabled && sdf > truncation_distance_) {
    return;
  }

  // Compute the weight of the measurement
  float observation_weight;
  {
    // Set the sign of the measurement weight
    observation_weight = num_rays_intersecting_voxel;

    // NOTE: Scaling the weight by the inverse square depth is not yet
    //       supported, since this problem is ill-defined for LiDAR

    // Apply weight drop-off if appropriate
    const voxblox::FloatingPoint dropoff_epsilon = voxel_size_;
    if (config_.use_weight_dropoff && sdf < -dropoff_epsilon) {
      observation_weight = observation_weight * (truncation_distance_ + sdf) / (truncation_distance_ - dropoff_epsilon);
      observation_weight = std::max(observation_weight, 0.0f);
    }

    // Apply sparsity compensation if appropriate
    if (config_.sparsity_compensation_factor != 1.0) {
      if (std::abs(sdf) < truncation_distance_) {
        observation_weight *= config_.sparsity_compensation_factor;
      }
    }
  }

  // Truncate the new total voxel weight according to the max weight
  const float new_voxel_weight = std::min(tsdf_voxel->weight + observation_weight, config_.max_weight);

  // Store the updated voxel weight and distance
  tsdf_voxel->distance =
      (tsdf_voxel->distance * tsdf_voxel->weight + std::min(truncation_distance_, sdf) * observation_weight)
          / new_voxel_weight;
  tsdf_voxel->weight = new_voxel_weight;
}

template<typename T>
Point ProjectiveMutliTSDFIntegrator::imageToBearing(const T h, const T w) {
  Point bearing;
  if (config_.sensor_is_lidar) {
    // lidar is input in sensor frame (x-fwd, y-left, z-up)
    double altitude_angle =
        c_vertical_fov_rad_ * (1.0 / 2.0 - h / (config_.sensor_vertical_resolution - 1.0));
    double azimuth_angle =
        2.0 * M_PI * static_cast<double>(w) / config_.sensor_horizontal_resolution;

    bearing.x() = std::cos(altitude_angle) * std::cos(azimuth_angle);
    bearing.y() = -std::cos(altitude_angle) * std::sin(azimuth_angle);
    bearing.z() = std::sin(altitude_angle);
  } else {
    // depth camera is input in camera frame (x-right, y-down, z-fwd)
    bearing.x() = w - config_.sensor_horizontal_resolution / 2;
    bearing.y() = h - config_.sensor_vertical_resolution / 2;
    bearing.z() = c_focal_length_;
    bearing = bearing.normalized();
  }
  return bearing;
}

template<typename T>
bool ProjectiveMutliTSDFIntegrator::bearingToImage(const Point &b_C_normalized, T *h, T *w) {
  CHECK_NOTNULL(h);
  CHECK_NOTNULL(w);

  if (config_.sensor_is_lidar) {
    // lidar
    double altitude_angle = std::asin(b_C_normalized.z());
    *h = static_cast<T>((config_.sensor_vertical_resolution - 1.0) *
        (1.0 / 2.0 - altitude_angle / c_vertical_fov_rad_));
    if (*h < 0 || config_.sensor_vertical_resolution - 1 < *h) {
      return false;
    }

    double azimuth_angle;
    if (b_C_normalized.x() > 0) {
      if (b_C_normalized.y() > 0) {
        azimuth_angle =
            2.0 * M_PI + std::atan(-b_C_normalized.y() / b_C_normalized.x());
      } else {
        azimuth_angle = std::atan(-b_C_normalized.y() / b_C_normalized.x());
      }
    } else {
      azimuth_angle = M_PI + std::atan(-b_C_normalized.y() / b_C_normalized.x());
    }
    *w = static_cast<T>(config_.sensor_horizontal_resolution * azimuth_angle / (2.0 * M_PI));
    if (*w < 0) {
      *w += config_.sensor_horizontal_resolution;
    }

    return (0 < *w && *w < config_.sensor_horizontal_resolution - 1);
  } else {
    // depth cam
    *w = static_cast<T>(
        b_C_normalized.x() * c_focal_length_ / std::sqrt(1.0 - std::pow(b_C_normalized.x(), 2.0))
            + config_.sensor_horizontal_resolution / 2);
    if (*w < 0 || *w >= config_.sensor_horizontal_resolution) {
      return false;
    }
    *h = static_cast<T>(
        b_C_normalized.y() * c_focal_length_ / std::sqrt(1.0 - std::pow(b_C_normalized.y(), 2.0))
            + config_.sensor_vertical_resolution / 2);
    return *h >= 0 && *h < config_.sensor_vertical_resolution;
  }
}

float ProjectiveMutliTSDFIntegrator::interpolate(const float h, const float w) {
  // nearest neighbor lookup
  return range_image_(std::round(h), std::round(w));
}

}  // namespace panoptic_mapping
