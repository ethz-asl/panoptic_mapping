#ifndef PANOPTIC_MAPPING_CORE_SUBMAP_H_
#define PANOPTIC_MAPPING_CORE_SUBMAP_H_

#include <memory>

#include <voxblox/core/layer.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox_msgs/Mesh.h>

#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/Submap.pb.h"

namespace panoptic_mapping {

class Submap {
 public:
  Submap(int id, double voxel_size, int voxels_per_side);

  // io
  void getProto(SubmapProto *proto) const;
  bool saveToStream(std::fstream *outfile_ptr) const;
  static std::unique_ptr<Submap> loadFromStream(std::fstream *proto_file_ptr, uint64_t *tmp_byte_offset_ptr);

  // const accessors
  int getID() const { return id_; }
  const std::string& getFrameName() const { return frame_name_; }
  const Color &getColor() const { return color_; }
  const voxblox::Layer<TsdfVoxel> &getTsdfLayer() const { return *tsdf_layer_; }
  const voxblox::MeshLayer &getMeshLayer() const { return *mesh_layer_; }
  const Transformation &getT_M_S() const { return T_M_S_; }
  const Transformation &getT_S_M() const { return T_M_S_inv_; }
  bool isObserved() const { return is_observed_; }

  // modifying accessors
  voxblox::Layer<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }
  voxblox::MeshLayer* getMeshLayerPtr() { return mesh_layer_.get(); }

  // setters
  void setColor(const Color &color) { color_ = color; }
  void setT_M_S(const Transformation& T_M_S);
  void setIsObserved(bool is_observed) { is_observed_ = is_observed; }

 private:
  const int id_;
  std::shared_ptr<voxblox::Layer<TsdfVoxel>> tsdf_layer_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
  voxblox::Transformation T_M_S_;   // transform mission to submap
  voxblox::Transformation T_M_S_inv_;
  voxblox::Color color_;    // color to use for visualizers
  std::string frame_name_;
  bool is_observed_;
};

} // namespace panoptic_mapping

#endif //PANOPTIC_MAPPING_CORE_SUBMAP_H_
