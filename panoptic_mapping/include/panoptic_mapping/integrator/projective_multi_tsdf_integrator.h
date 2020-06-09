#ifndef PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_MULTI_TSDF_INTEGRATOR_H_
#define PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_MULTI_TSDF_INTEGRATOR_H_

#include <memory>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/integrator/pointcloud_integrator_base.h"

namespace panoptic_mapping {

/**
 * Multi-layer TSDF-Integrator. Based on the voxblox projective tsdf integrator and hacked together for testing.
 */
class ProjectiveMutliTSDFIntegrator : public PointcloudIntegratorBase {
 public:
  struct Config : PointcloudIntegratorBase::Config {
    // sensor model
    int horizontal_resolution = 0.0;
    int vertical_resolution = 0.0;
    float vertical_fov_deg = 0.0;
    float min_ray_length_m = 0.1;
    float max_ray_length_m = 5.0;
    bool voxel_carving_enabled = false;
    int integrator_threads = 8;
    bool use_weight_dropoff = true;
    float max_weight = 10000.0;
  };

  ProjectiveMutliTSDFIntegrator() = default;
  virtual ~ProjectiveMutliTSDFIntegrator() = default;

  // initialization utility
  void setupFromConfig(PointcloudIntegratorBase::Config *config) override;

  // process a and integrate a pointcloud
  void processPointcloud(SubmapCollection *submaps,
                         const Transformation &T_M_C,
                         const Pointcloud &pointcloud,
                         const Colors &colors,
                         const std::vector<int> &ids) override;

 protected:
  // fixed settings
  Config config_;
  float vertical_fov_rad_;

  // cached pointcloud data, reset on readPointcloud()
  Eigen::MatrixXf range_image_;
  Eigen::MatrixXi id_image_;
  Transformation T_W_C_;
  std::vector<Point> bearing_points_;  // to find touched blocks
  std::vector<float> distances_;

  // cached tsdf_layer data, reset on setTsdfIntegrationLayer()
  voxblox::Layer<voxblox::TsdfVoxel> *layer_;
  size_t num_voxels_per_block_;
  float voxel_size_;
  float ray_intersections_per_distance_squared_;
  float truncation_distance_;

  // methods
  void readPointcloud(const Transformation &T_W_C,
                      const Pointcloud &pointcloud,
                      const Colors &colors,
                      const std::vector<int> &ids);
  void integratePointcloudToSubmaps(Submap *submap);
  void setTsdfIntegrationLayer(voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer);
  void findTouchedBlocks(voxblox::IndexSet *touched_block_indices, voxblox::IndexSet *containing_blocks_indices);

  // integration
  void updateTsdfBlocks(const Transformation &T_G_C,
                        const Eigen::MatrixXf &range_image,
                        const voxblox::IndexSet &touched_block_indices);
  void updateTsdfVoxel(const Transformation &T_G_C,
                              const Eigen::MatrixXf &range_image,
                              const Point &t_C_voxel, voxblox::TsdfVoxel *tsdf_voxel);
  float interpolate(const float h, const float w);

  // image-bearing conversions
  template<typename T>
  Point imageToBearing(const T h, const T w);
  template<typename T>
  bool bearingToImage(const Point &b_C_normalized, T *h, T *w);

};
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_POINTCLOUD_INTEGRATOR_H_
