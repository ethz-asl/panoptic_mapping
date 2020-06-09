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
class ProjectiveMutliTSDFIntegrator {
 public:
  struct Config{
    // sensor model
    int horizontal_resolution = 0;
    int vertical_resolution = 0;
    double vertical_fov_deg = 0;
    float min_ray_length_m = 0;
    float max_ray_length_m = 5;
    bool voxel_carving_enabled = false;
    int integrator_threads = 8;
  };

  ProjectiveMutliTSDFIntegrator() = default;
  virtual ~ProjectiveMutliTSDFIntegrator() = default;

  // initialization utility
  static Config getConfigFromRos(const ros::NodeHandle &node_handle);

  // read in a pointcloud with the Transform of the world to the camera
  void readPointcloud(const Transformation &T_W_C, const Pointcloud &pointcloud, const Colors &colors, const std::vector<int> &ids);

  // integrate the last read in cloud to a given submap
  void integratePointcloudToSubmaps(std::unordered_map<int,);

 protected:
  // fixed settings
  Config config_;
  double vertical_fov_rad_;

  // pointcloud data
  Eigen::MatrixXf range_image_;
  Eigen::MatrixXi id_image_;
  Transformation T_W_C_;
  std::vector<Point> bearing_points_;  // to find touched blocks
  std::vector<float> distances_;

  // tsdf_layer data
  voxblox::Layer<voxblox::TsdfVoxel>* layer_;
  size_t num_voxels_per_block_;
  double ray_intersections_per_distance_squared_;
  float truncation_distance_;

  // methods
  void setTsdfIntegrationLayer(voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer);
  void findTouchedBlocks(voxblox::IndexSet *touched_block_indices);


  // stuff copied from voxblox
  void updateTsdfBlocks(const Transformation& T_G_C,
                        const Eigen::MatrixXf& range_image,
                        const voxblox::IndexSet& touched_block_indices,
                        const bool deintegrate = false);
  inline void updateTsdfVoxel(const Transformation& T_G_C,
                              const Eigen::MatrixXf& range_image,
                              const Point& t_C_voxel, voxblox::TsdfVoxel* tsdf_voxel,
                              const bool deintegrate = false);
  inline float interpolate(const Eigen::MatrixXf& range_image, const float h,const float w);

  template <typename T>
  Point imageToBearing(const T h, const T w);
  template <typename T>
  bool bearingToImage(const Point& b_C_normalized, T* h, T* w);

};
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_POINTCLOUD_INTEGRATOR_H_
