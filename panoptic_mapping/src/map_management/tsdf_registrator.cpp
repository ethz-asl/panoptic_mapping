#include "panoptic_mapping/map_management/tsdf_registrator.h"

#include <algorithm>
#include <future>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <voxblox/integrator/merge_integration.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "panoptic_mapping/common/index_getter.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

void TsdfRegistrator::Config::checkParams() const {
  checkParamNE(error_threshold, 0.f, "error_threshold");
  checkParamGE(min_voxel_weight, 0.f, "min_voxel_weight");
  checkParamGE(match_rejection_points, 0, "match_rejection_points");
  checkParamGE(match_rejection_percentage, 0.f, "match_rejection_percentage");
  checkParamLE(match_rejection_percentage, 1.f, "match_rejection_percentage");
  checkParamGE(match_acceptance_points, 0, "match_acceptance_points");
  checkParamGE(match_acceptance_percentage, 0.f, "match_acceptance_percentage");
  checkParamLE(match_acceptance_percentage, 1.f, "match_acceptance_percentage");
  if (normalize_by_voxel_weight) {
    checkParamGT(normalization_max_weight, 0.f, "normalization_max_weight");
  }
  checkParamGT(integration_threads, 0, "integration_threads");
}

void TsdfRegistrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("min_voxel_weight", &min_voxel_weight);
  setupParam("error_threshold", &error_threshold);
  setupParam("match_rejection_points", &match_rejection_points);
  setupParam("match_rejection_percentage", &match_rejection_percentage);
  setupParam("match_acceptance_points", &match_acceptance_points);
  setupParam("match_acceptance_percentage", &match_acceptance_percentage);
  setupParam("normalize_by_voxel_weight", &normalize_by_voxel_weight);
  setupParam("normalization_max_weight", &normalization_max_weight);
  setupParam("integration_threads", &integration_threads);
}

TsdfRegistrator::TsdfRegistrator(const Config& config)
    : config_(config.checkValid()) {}

void TsdfRegistrator::checkSubmapCollectionForChange(
    SubmapCollection* submaps) const {
  auto t_start = std::chrono::high_resolution_clock::now();
  std::string info;

  // Check all inactive maps for alignment with the currently active ones.
  std::vector<int> id_list;
  for (const Submap& submap : *submaps) {
    if (!submap.isActive() && submap.getLabel() != PanopticLabel::kFreeSpace &&
        !submap.getIsoSurfacePoints().empty()) {
      id_list.emplace_back(submap.getID());
    }
  }

  // Perform change detection in parallel.
  SubmapIndexGetter index_getter(id_list);
  std::vector<std::future<std::string>> threads;
  for (int i = 0; i < config_.integration_threads; ++i) {
    threads.emplace_back(
        std::async(std::launch::async, [this, &index_getter, submaps]() {
          int index;
          std::string info;
          while (index_getter.getNextIndex(&index)) {
            info += this->checkSubmapForChange(*submaps,
                                               submaps->getSubmapPtr(index));
          }
          return info;
        }));
  }

  // Join all threads.
  for (auto& thread : threads) {
    info += thread.get();
  }
  auto t_end = std::chrono::high_resolution_clock::now();

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Performed change detection in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << (config_.verbosity < 3 || info.empty() ? "ms." : "ms:" + info);
}

std::string TsdfRegistrator::checkSubmapForChange(
    const SubmapCollection& submaps, Submap* submap) const {
  // Check overlapping submaps for conflicts or matches.
  for (const Submap& other : submaps) {
    if (!other.isActive() ||
        !submap->getBoundingVolume().intersects(other.getBoundingVolume())) {
      continue;
    }

    // Note(schmluk): Exclude free space for thin structures. Although there's
    // potentially a nicer way of solving this.
    if (other.getLabel() == PanopticLabel::kFreeSpace &&
        submap->getConfig().voxel_size < other.getConfig().voxel_size * 0.5) {
      continue;
    }

    bool submaps_match;
    if (submapsConflict(*submap, other, &submaps_match)) {
      // No conflicts allowed.
      if (submap->getChangeState() != ChangeState::kAbsent) {
        submap->setChangeState(ChangeState::kAbsent);
      }
      std::stringstream info;
      info << "\nSubmap " << submap->getID() << " (" << submap->getName()
           << ") conflicts with submap " << other.getID() << " ("
           << other.getName() << ").";
      return info.str();
    } else if (submap->getClassID() == other.getClassID() && submaps_match) {
      // Semantically and geometrically match.
      submap->setChangeState(ChangeState::kPersistent);
    }
  }
  return "";
}

bool TsdfRegistrator::submapsConflict(const Submap& reference,
                                      const Submap& other,
                                      bool* submaps_match) const {
  // Reference is the finished submap (with Iso-surfce-points) that is
  // compared to the active submap other.
  Transformation T_O_R = other.getT_S_M() * reference.getT_M_S();
  const float rejection_count =
      config_.normalize_by_voxel_weight
          ? std::numeric_limits<float>::max()
          : std::max(static_cast<float>(config_.match_rejection_points),
                     config_.match_rejection_percentage *
                         reference.getIsoSurfacePoints().size());
  const float rejection_distance =
      config_.error_threshold > 0.f
          ? config_.error_threshold
          : config_.error_threshold * -other.getTsdfLayer().voxel_size();
  float conflicting_points = 0.f;
  float matched_points = 0.f;
  float total_weight = 0.f;
  voxblox::Interpolator<TsdfVoxel> interpolator(&(other.getTsdfLayer()));

  // Check for disagreement.
  float distance, weight;
  for (const auto& point : reference.getIsoSurfacePoints()) {
    if (getDistanceAndWeightAtPoint(&distance, &weight, point, T_O_R,
                                    interpolator)) {
      // Compute the weight to be used for counting.
      if (config_.normalize_by_voxel_weight) {
        weight = computeCombinedWeight(weight, point.weight);
        total_weight += weight;
      } else {
        weight = 1.f;
      }

      // Count.
      if (other.getLabel() == PanopticLabel::kFreeSpace) {
        if (distance >= rejection_distance) {
          conflicting_points += weight;
        }
      } else {
        // Check for class belonging.

        if (other.hasClassLayer()) {
          const ClassVoxel* class_voxel =
              other.getClassLayer().getVoxelPtrByCoordinates(point.position);
          if (class_voxel) {
            if (!class_voxel->belongsToSubmap()) {
              distance = other.getConfig().truncation_distance;
            }
          }
        }
        if (distance <= -rejection_distance) {
          conflicting_points += weight;
        } else if (distance <= rejection_distance) {
          matched_points += weight;
        }
      }

      if (conflicting_points > rejection_count) {
        // If the rejection count is known and reached submaps conflict.
        if (submaps_match) {
          *submaps_match = false;
        }
        return true;
      }
    }
  }
  // Evaluate the result.
  if (config_.normalize_by_voxel_weight) {
    const float rejection_weight =
        std::max(static_cast<float>(config_.match_rejection_points) /
                     reference.getIsoSurfacePoints().size(),
                 config_.match_rejection_percentage) *
        total_weight;
    if (conflicting_points > rejection_weight) {
      if (submaps_match) {
        *submaps_match = false;
      }
      return true;
    } else if (submaps_match) {
      const float acceptance_weight =
          std::max(static_cast<float>(config_.match_acceptance_points) /
                       reference.getIsoSurfacePoints().size(),
                   config_.match_acceptance_percentage) *
          total_weight;
      if (matched_points > acceptance_weight) {
        *submaps_match = true;
      } else {
        *submaps_match = false;
      }
    }
  } else {
    if (submaps_match) {
      const float acceptance_count =
          std::max(static_cast<float>(config_.match_acceptance_points),
                   config_.match_acceptance_percentage *
                       reference.getIsoSurfacePoints().size());
      *submaps_match = matched_points > acceptance_count;
    }
  }
  return false;
}  // namespace panoptic_mapping

bool TsdfRegistrator::getDistanceAndWeightAtPoint(
    float* distance, float* weight, const IsoSurfacePoint& point,
    const Transformation& T_P_S,
    const voxblox::Interpolator<TsdfVoxel>& interpolator) const {
  // Check minimum input point weight.
  if (point.weight < config_.min_voxel_weight) {
    return false;
  }

  // Try to interpolate the voxel in the  map.
  // NOTE(Schmluk): This also interpolates color etc, but otherwise the
  // interpolation lookup has to be done twice. Getting only what we want is
  // also in voxblox::interpolator but private, atm not performance critical.
  TsdfVoxel voxel;
  const Point position = T_P_S * point.position;
  if (!interpolator.getVoxel(position, &voxel, true)) {
    return false;
  }
  if (voxel.weight < config_.min_voxel_weight) {
    return false;
  }
  *distance = voxel.distance;
  *weight = voxel.weight;
  return true;
}

float TsdfRegistrator::computeCombinedWeight(float w1, float w2) const {
  if (w1 <= 0.f || w2 <= 0.f) {
    return 0.f;
  } else if (w1 >= config_.normalization_max_weight &&
             w2 >= config_.normalization_max_weight) {
    return 1.f;
  } else {
    return std::sqrt(std::min(w1 / config_.normalization_max_weight, 1.f) *
                     std::min(w2 / config_.normalization_max_weight, 1.f));
  }
}

void TsdfRegistrator::mergeMatchingSubmaps(SubmapCollection* submaps) {
  // Merge all submaps of identical Instance ID into one.
  // TODO(schmluk): This is a preliminary function for prototyping, update
  // this.
  submaps->updateInstanceToSubmapIDTable();
  int merged_maps = 0;
  for (const auto& instance_submaps : submaps->getInstanceToSubmapIDTable()) {
    const auto& ids = instance_submaps.second;
    Submap* target;
    for (auto it = ids.begin(); it != ids.end(); ++it) {
      if (it == ids.begin()) {
        target = submaps->getSubmapPtr(*it);
        continue;
      }
      // Merging.
      merged_maps++;
      voxblox::mergeLayerAintoLayerB(submaps->getSubmap(*it).getTsdfLayer(),
                                     target->getTsdfLayerPtr().get());
      submaps->removeSubmap(*it);
    }
    // Set the updated flags of the changed layer.
    voxblox::BlockIndexList block_list;
    target->getTsdfLayer().getAllAllocatedBlocks(&block_list);
    for (auto& index : block_list) {
      target->getTsdfLayerPtr()->getBlockByIndex(index).setUpdatedAll();
    }
  }
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Merged " << merged_maps << " submaps.";
}

}  // namespace panoptic_mapping
