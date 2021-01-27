#include "panoptic_mapping/integrator/naive_integrator.h"

#include <vector>

namespace panoptic_mapping {

NaiveIntegrator::Config NaiveIntegrator::Config::isValid() const {
  if (voxblox_integrator_type != "simple" &&
      voxblox_integrator_type != "merged" &&
      voxblox_integrator_type != "fast" &&
      voxblox_integrator_type != "projective") {
    LOG(FATAL) << "Unknown voxblox_integrator_type '" << voxblox_integrator_type
               << "'.";
  }
  return Config(*this);
}

NaiveIntegrator::NaiveIntegrator(const Config& config)
    : config_(config.isValid()) {}

void NaiveIntegrator::processInput(SubmapCollection* submaps, InputData * input) {

  //NOTE(schmluk): This is legacy code for running the standard voxblox
  // integrators. If this is to be used all the data is in the input, just need
  // to convert the images to pointcloud.

  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));

  /*
  CHECK_NOTNULL(submaps);
  CHECK_EQ(ids.size(), pointcloud.size());
  CHECK_EQ(ids.size(), colors.size());

  // Segment the pointcloud by id
  std::vector<voxblox::Pointcloud> cloud_v;
  std::vector<voxblox::Colors> color_v;
  std::vector<int> id_v;

  for (size_t i = 0; i < ids.size(); ++i) {
    auto it = std::find(id_v.begin(), id_v.end(), ids[i]);
    size_t index = it - id_v.begin();
    if (it == id_v.end()) {
      // allocate new partial cloud
      id_v.emplace_back(ids[i]);
      cloud_v.emplace_back(voxblox::Pointcloud());
      color_v.emplace_back(voxblox::Colors());
    }
    cloud_v.at(index).push_back(pointcloud[i]);
    color_v.at(index).push_back(colors[i]);
  }

  // integrate each partial pointcloud to each submap
  for (size_t i = 0; i < id_v.size(); ++i) {
    if (!submaps->submapIdExists(id_v[i])) {
      // all submaps should already be allocated
      LOG(WARNING) << "Failed to integrate pointcloud to submap with ID '"
                   << id_v[i] << "': submap does not exist.";
      continue;
    }
    if (!tsdf_integrator_) {
      tsdf_integrator_ = voxblox::TsdfIntegratorFactory::create(
          config_.voxblox_integrator_type, config_.voxblox_integrator_config,
          submaps->getSubmapPtr(id_v[i])->getTsdfLayerPtr().get());
    } else {
      tsdf_integrator_->setLayer(
          submaps->getSubmapPtr(id_v[i])->getTsdfLayerPtr().get());
    }
    tsdf_integrator_->integratePointCloud(T_M_C, cloud_v[i], color_v[i]);
  }*/
}

}  // namespace panoptic_mapping
