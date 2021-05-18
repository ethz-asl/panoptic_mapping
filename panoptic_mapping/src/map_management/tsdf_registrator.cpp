#include "panoptic_mapping/map_management/tsdf_registrator.h"

#include <algorithm>
#include <utility>
#include <vector>

#include <voxblox/integrator/merge_integration.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

void TsdfRegistrator::Config::checkParams() const {
  checkParamNE(error_threshold, 0.f, "error_threshold");
  checkParamGE(min_voxel_weight, 0.f, "min_voxel_weight");
  checkParamGE(match_rejection_points, 0, "match_rejection_points");
  checkParamGE(match_rejection_percentage, 0.f, "match_rejection_percentage");
  checkParamGE(match_acceptance_points, 0, "match_acceptance_points");
  checkParamGE(match_acceptance_percentage, 0.f, "match_acceptance_percentage");
}

void TsdfRegistrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("min_voxel_weight", &min_voxel_weight);
  setupParam("error_threshold", &error_threshold);
  setupParam("error_threshold", &error_threshold);
  setupParam("match_rejection_percentage", &match_rejection_percentage);
  setupParam("match_acceptance_points", &match_acceptance_points);
  setupParam("match_acceptance_percentage", &match_acceptance_percentage);
  setupParam("normalize_by_voxel_weight", &normalize_by_voxel_weight);
  setupParam("allow_multiple_matches", &allow_multiple_matches);
}

TsdfRegistrator::TsdfRegistrator(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void TsdfRegistrator::checkSubmapCollectionForChange(
    const SubmapCollection& submaps) const {
  auto t_start = std::chrono::high_resolution_clock::now();
  std::stringstream info;

  // Check all inactive maps for alignment with the currently active ones.
  for (const auto& submap : submaps) {
    if (submap->isActive() || submap->getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }

    // Check overlapping submaps for conflicts or matches and store best match.
    std::vector<std::pair<int, float>> matching_ids_points;
    if (!config_.allow_multiple_matches) {
      matching_ids_points = {{0, -1.f}};  // empty initializer.
    }
    for (const auto& other : submaps) {
      if (!other->isActive() ||
          !submap->getBoundingVolume().intersects(other->getBoundingVolume())) {
        continue;
      }

      float matching_points;
      if (submapsConflict(*submap, *other, &matching_points)) {
        // No conflicts allowed, update also the matched submap if it
        // exists.
        if (submap->getChangeState() != ChangeState::kAbsent) {
          info << "\nSubmap " << submap->getID() << " (" << submap->getName()
               << ") conflicts with submap " << other->getID() << " ("
               << other->getName() << ")";
          if (submap->getChangeState() == ChangeState::kPersistent) {
            if (!config_.allow_multiple_matches) {
              for (const auto& matched : submaps) {
                if (matched->isActive() &&
                    matched->getInstanceID() == submap->getInstanceID()) {
                  matched->setChangeState(ChangeState::kNew);
                  info << ", was matched with submap " << matched->getID()
                       << " (" << matched->getName() << ")";
                  break;
                }
              }
              // Get a new instance id.
              submap->setInstanceID(InstanceID());
            }
          }
          submap->setChangeState(ChangeState::kAbsent);
          info << ".";
        }
        break;
      } else if (submap->getClassID() == other->getClassID()) {
        // Semantically match, so could be a candidate.
        if (config_.allow_multiple_matches) {
          matching_ids_points.emplace_back(other->getID(), matching_points);
        } else if (matching_ids_points[0].second < matching_points) {
          matching_ids_points[0] = {other->getID(), matching_points};
        }
      }
    }

    // Check for sufficient alignment and match the most suitable
    // candidate.
    const float acceptance_count =
        std::min(static_cast<float>(config_.match_acceptance_points),
                 config_.match_acceptance_percentage *
                     submap->getIsoSurfacePoints().size());
    if (!config_.allow_multiple_matches) {
      if (matching_ids_points[0].second > acceptance_count) {
        Submap* other = submaps.getSubmapPtr(matching_ids_points[0].first);
        if (other->getInstanceID() != submap->getInstanceID()) {
          // Geometry and semantic class match, it's a new match.
          submap->setChangeState(ChangeState::kPersistent);
          submap->setInstanceID(other->getInstanceID());
          other->setChangeState(ChangeState::kMatched);
          info << "\nSubmap " << submap->getID() << " (" << submap->getName()
               << ") was matched with submap " << other->getID() << " ("
               << other->getName() << ").";
        }
      }
    } else {
      LOG_FIRST_N(WARNING, 1) << "Multiple matches not yet supported.";
    }
  }
  auto t_end = std::chrono::high_resolution_clock::now();

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Performed change detection in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << (config_.verbosity < 3 || info.str().empty() ? "ms."
                                                      : "ms:" + info.str());
}

bool TsdfRegistrator::submapsConflict(const Submap& reference,
                                      const Submap& other,
                                      float* matching_points) const {
  // Reference is the finished submap (with Iso-surfce-points) that is compared
  // to the active submap other.
  Transformation T_O_R = other.getT_S_M() * reference.getT_M_S();
  const float rejection_count =
      std::min(static_cast<float>(config_.match_rejection_points),
               config_.match_rejection_percentage *
                   reference.getIsoSurfacePoints().size());
  const float rejection_distance =
      config_.error_threshold > 0
          ? config_.error_threshold
          : config_.error_threshold * -1.f * other.getTsdfLayer().voxel_size();
  float conflicting_points = 0;
  float matched_points = 0;
  voxblox::Interpolator<TsdfVoxel> interpolator(&(other.getTsdfLayer()));

  // Check for disagreement.
  float distance, weight;
  for (const auto& point : reference.getIsoSurfacePoints()) {
    if (getDistanceAndWeightAtPoint(&distance, &weight, point, T_O_R,
                                    interpolator)) {
      if (other.getLabel() == PanopticLabel::kFreeSpace) {
        if (distance >= rejection_distance) {
          conflicting_points += 1.f;
        }
      } else {
        if (distance <= -rejection_distance) {
          conflicting_points += 1.f;
        } else if (distance <= rejection_distance) {
          matched_points += 1.f;
        }
      }

      if (conflicting_points >= rejection_count) {
        return true;
      }
    }
  }
  if (matching_points) {
    *matching_points = matched_points;
  }
  return false;
}

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

void TsdfRegistrator::mergeMatchingSubmaps(SubmapCollection* submaps) {
  // Merge all submaps of identical Instance ID into one.
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