#include "panoptic_mapping/registration/tsdf_registrator.h"

#include <algorithm>

#include <voxblox/mesh/mesh_integrator.h>

#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/submap_collection.h"

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
}

TsdfRegistrator::TsdfRegistrator(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void TsdfRegistrator::computeIsoSurfacePoints(Submap* submap) const {
  // NOTE(schmluk): Currently all surface points are computed from scratch every
  // time, but since they are currently only computed when a submap is finished
  // it should be fine.
  CHECK_NOTNULL(submap);
  submap->getIsoSurfacePointsPtr()->clear();

  // Initialize the mesh layer with new points.
  const float voxel_size = submap->getTsdfLayer().voxel_size();
  const float block_size = submap->getTsdfLayer().block_size();
  voxblox::MeshLayer mesh_layer(block_size);

  // Setup the mesh integrator.
  voxblox::MeshIntegratorConfig mesh_integrator_config;
  mesh_integrator_config.use_color = false;
  mesh_integrator_config.min_weight =
      static_cast<float>(config_.min_voxel_weight);
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      mesh_integrator_config, submap->getTsdfLayer(), &mesh_layer);
  mesh_integrator.generateMesh(false, false);

  // Convert it into a connected mesh.
  Point origin{0, 0, 0};
  voxblox::Mesh connected_mesh(block_size, origin);
  mesh_layer.getConnectedMesh(&connected_mesh, 0.5 * voxel_size);

  // Create an interpolator to interpolate the vertex weights from the TSDF.
  voxblox::Interpolator<TsdfVoxel> interpolator(
      submap->getTsdfLayerPtr().get());

  // Extract the vertices.
  for (const auto& mesh_vertex_coordinates : connected_mesh.vertices) {
    // Try to interpolate the voxel weight.
    TsdfVoxel voxel;
    if (interpolator.getVoxel(mesh_vertex_coordinates, &voxel, true)) {
      CHECK_LE(voxel.distance, 1e-2 * voxel_size);

      // Store the isosurface vertex.
      submap->getIsoSurfacePointsPtr()->emplace_back(IsoSurfacePoint{
          mesh_vertex_coordinates, voxel.distance, voxel.weight});
    }
  }
}

void TsdfRegistrator::checkSubmapCollectionForChange(
    const SubmapCollection& submaps) const {
  auto t_start = std::chrono::high_resolution_clock::now();
  for (const auto& submap : submaps) {
    // Check all unobserved and persistent maps for changes.
    // NOTE(Schmluk): At the moment objects can't move so they stay absent
    // forever. Need to change this once PGO is supported.
    if (submap->getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }
    if (submap->getChangeDetectionData().state !=
            ChangeDetectionData::State::kUnobserved &&
        submap->getChangeDetectionData().state !=
            ChangeDetectionData::State::kPersistent) {
      continue;
    }

    // Find all potentially overlapping submaps and check for conflicts.
    for (const auto& other : submaps) {
      if (!other->isActive() ||
          !submap->getBoundingVolume().intersects(other->getBoundingVolume())) {
        continue;
      }
      ChangeDetectionData* change = submap->getChangeDetectionDataPtr();
      bool is_match = false;
      if (submapsConflict(*submap, *other, &is_match)) {
        // No conflicts allowed, update also the matched submap if it exists.
        if (change->state == ChangeDetectionData::State::kMatched) {
          submaps.getSubmapPtr(change->matched_submap_id)
              ->getChangeDetectionDataPtr()
              ->state = ChangeDetectionData::State::kNew;
        }
        change->state = ChangeDetectionData::State::kAbsent;
        break;
      } else if (submap->getClassID() == other->getClassID() && is_match) {
        // Geometry and semantic class match.
        change->state = ChangeDetectionData::State::kPersistent;
        change->matched_submap_id = other->getID();
        other->getChangeDetectionDataPtr()->state =
            ChangeDetectionData::State::kMatched;
        other->getChangeDetectionDataPtr()->matched_submap_id = submap->getID();
      }
    }
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Performed change detection in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << "ms.";
}

bool TsdfRegistrator::submapsConflict(const Submap& reference,
                                      const Submap& other,
                                      bool* is_match) const {
  // Reference is the finished submap (with Iso-surfce-points) that is compared
  // to the active submap other.
  const unsigned int rejection_count =
      std::min(config_.match_rejection_points,
               static_cast<int>(config_.match_rejection_percentage *
                                other.getIsoSurfacePoints().size()));
  const float rejection_distance =
      config_.error_threshold > 0
          ? config_.error_threshold
          : config_.error_threshold * -1.f * other.getTsdfLayer().voxel_size();
  const float direction_multiplier =
      other.getLabel() == PanopticLabel::kFreeSpace ? 1.f : -1.f;
  unsigned int conflicting_points = 0;
  unsigned int matching_points = 0;
  voxblox::Interpolator<TsdfVoxel> interpolator(&(other.getTsdfLayer()));

  // Check for disagreement.
  float distance;
  for (const auto& point : reference.getIsoSurfacePoints()) {
    if (getDistanceAtPoint(&distance, point, interpolator)) {
      if (distance * direction_multiplier > rejection_distance) {
        conflicting_points++;
        if (conflicting_points >= rejection_count) {
          return true;
        }
      } else {
        matching_points++;
      }
    }
  }
  if (is_match) {
    const unsigned int acceptance_count =
        std::min(config_.match_acceptance_points,
                 static_cast<int>(config_.match_acceptance_percentage *
                                  other.getIsoSurfacePoints().size()));
    *is_match = matching_points >= acceptance_count;
  }
  return false;
}

bool TsdfRegistrator::getDistanceAtPoint(
    float* distance, const IsoSurfacePoint& point,
    const voxblox::Interpolator<TsdfVoxel>& interpolator) const {
  // Check minimum input point weight.
  if (point.weight < config_.min_voxel_weight) {
    return false;
  }
  // Try to interpolate the voxel in the  map.
  // NOTE(Schmluk): This also interpolates color etc, but otherwise the
  // interpolation lookup has to be done twice. Getting only what we want is
  // also in voxblox::interpolator but private. Atm not performance critical.
  TsdfVoxel voxel;
  if (!interpolator.getVoxel(point.position, &voxel, true)) {
    return false;
  }
  if (voxel.weight < config_.min_voxel_weight) {
    return false;
  }
  *distance = voxel.distance;
  return true;
}

}  // namespace panoptic_mapping
