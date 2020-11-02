#include "panoptic_mapping/registration/tsdf_registrator.h"

#include <voxblox/mesh/mesh_integrator.h>

namespace panoptic_mapping {

void TsdfRegistrator::Config::checkParams() const {
  checkParamNE(error_threshold, 0.0, "error_threshold");
  checkParamGT(min_number_of_matching_points, 0,
               "min_number_of_matching_points");
}

void TsdfRegistrator::Config::setupParamsAndPrinting() {
  setupParam("min_voxel_weight", &min_voxel_weight);
  setupParam("error_threshold", &error_threshold);
  setupParam("weight_error_by_tsdf_weight", &weight_error_by_tsdf_weight);
  setupParam("min_number_of_matching_points", &min_number_of_matching_points);
  setupParam("match_rejection_percentage", &match_rejection_percentage);
}

TsdfRegistrator::TsdfRegistrator(const Config& config)
    : config_(config.checkValid()) {}

void TsdfRegistrator::computeIsoSurfacePoints(Submap* submap) const {
  // NOTE(schmluk): Currently all surface points are computed from scratch every
  // time, but since they are currently only computed when a submap is finished
  // it should be fine.
  CHECK_NOTNULL(submap);
  submap->getIsoSurfacePointsPtr()->clear();

  // Initialize the mesh layer with new points
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
      submap->getIsoSurfacePointsPtr()->emplace_back(Submap::IsoSurfacePoint{
          mesh_vertex_coordinates, voxel.distance, voxel.weight});
    }
  }
}

TsdfRegistrator::ChangeDetectionResult
TsdfRegistrator::computeSurfaceDifference(Submap* reference,
                                          Submap* other) const {
  // NOTE(schmluk): 'reference' is the submap under construction, 'other' is the
  // finished submap containing the isosurface points.
  CHECK_NOTNULL(reference);
  CHECK_NOTNULL(other);

  // Set up data and intzerpolator.
  const double distance_threshold =
      config_.error_threshold > 0.0
          ? config_.error_threshold
          : config_.error_threshold * -0.5 *
                (reference->getTsdfLayer().voxel_size() +
                 other->getTsdfLayer().voxel_size());
  ChangeDetectionResult result;
  double squared_error = 0.0;
  double total_weight = 0.0;
  voxblox::Interpolator<TsdfVoxel> interpolator(
      reference->getTsdfLayerPtr().get());

  // Iterate through all points in other.
  for (const auto& point : other->getIsoSurfacePoints()) {
    // Check minimum weight other point weight.
    if (point.weight < config_.min_voxel_weight) {
      continue;
    }

    // Try to interpolate the voxel in the reference map.
    TsdfVoxel voxel;
    if (!interpolator.getVoxel(point.position, &voxel, true)) {
      result.number_of_checked_points++;
      continue;
    }

    // Check minimum reference weight.
    if (voxel.weight < config_.min_voxel_weight) {
      continue;
    }

    // The distance on the surface should be 0.
    double error = std::abs(voxel.distance);
    double weight = 1.0;
    if (config_.weight_error_by_tsdf_weight) {
      weight = voxel.weight * point.weight;
    }
    squared_error += weight * error * error;
    total_weight += weight;
    if (error <= distance_threshold) {
      result.points_within_threshold++;
    }
    result.number_of_observed_points++;
    result.number_of_checked_points++;
  }
  result.distance_rmse =
      total_weight != 0.0 ? std::sqrt(squared_error / total_weight) : 0.0;
  return result;
}

void TsdfRegistrator::checkSubmapCollectionForChange(
    const SubmapCollection& submaps) {
  std::cout << " ===== Checking for change ===== " << std::endl;

  // TODO(schmluk): Extend to track for deleted/deactivated submaps later.
  for (const auto& submap : submaps) {
    std::cout << submap->getID() << ": active " << submap->isActive()
              << ", matched " << submap->getChangeDetectionData().is_matched
              << "-" << submap->getChangeDetectionData().matching_submap_id
              << " n_iso_p: " << submap->getIsoSurfacePoints().size()
              << std::endl;

    // Only match all active submaps vs inactive ones.
    if (submap->isActive()) {
      Submap::ChangeDetectionData* change_data =
          submap->getChangeDetectionDataPtr();
      if (change_data->is_matched) {
        // Check if the match still holds.
        Submap* matched_submap =
            submaps.getSubmapPtr(change_data->matching_submap_id);
        ChangeDetectionResult change_detection_result =
            computeSurfaceDifference(submap.get(), matched_submap);
        if (!isMatch(change_detection_result)) {
          std::cout << "Previous match deleted." << std::endl;
          change_data->is_matched = false;
          matched_submap->getChangeDetectionDataPtr()->is_matched = false;
        }
      } else {
        // Compare against all inactive submaps of the same class for a match.
        for (const auto& candidate : submaps) {
          if (candidate->isActive()) {
            continue;
          }

          if (candidate->getClassID() == submap->getClassID()) {
            ChangeDetectionResult change_detection_result =
                computeSurfaceDifference(submap.get(), candidate.get());
            if (isMatch(change_detection_result)) {
              std::cout << "Found new match to: " << candidate->getID()
                        << std::endl;
              change_data->is_matched = true;
              change_data->matching_submap_id = candidate->getID();
              candidate->getChangeDetectionDataPtr()->is_matched = true;
              candidate->getChangeDetectionDataPtr()->matching_submap_id =
                  submap->getID();
            }
          }
        }
      }
    }
  }
}

bool TsdfRegistrator::isMatch(const ChangeDetectionResult& change_data) const {
  // Currently just check whether minimum number of points is observed and the
  // majority of points matches.
  if (change_data.number_of_observed_points <
      config_.min_number_of_matching_points) {
    return false;
  }
  if (static_cast<double>(change_data.points_within_threshold) /
          static_cast<double>(change_data.number_of_observed_points) <
      config_.match_rejection_percentage) {
    return false;
  }
  return true;
}

void TsdfRegistrator::resetChangeTracking() {}

}  // namespace panoptic_mapping
