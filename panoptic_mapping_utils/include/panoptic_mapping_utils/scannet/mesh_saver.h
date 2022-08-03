#ifndef PANOPTIC_MAPPING_UTILS_SCANNET_MESHSAVER_H_
#define PANOPTIC_MAPPING_UTILS_SCANNET_MESHSAVER_H_

#include <string>
#include <vector>

#include <ros/ros.h>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/mesh/mesh.h"
#include "voxblox/mesh/mesh_utils.h"
#include "voxblox/io/mesh_ply.h"
#include <voxblox_msgs/MultiMesh.h>
#include <voxblox_ros/mesh_vis.h>

namespace panoptic_mapping {

class MeshSaver {
 public:
  MeshSaver(const ros::NodeHandle& nh);
  virtual ~MeshSaver() = default;
  void gotMeshCallback(const voxblox_msgs::MultiMesh& msg);

 private:
  void setupRos();

  ros::NodeHandle nh_;

  ros::Subscriber mesh_sub_;
};

}  // namespace panoptic_mapping

#endif
