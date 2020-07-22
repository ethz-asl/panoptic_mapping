#ifndef PANOPTIC_MAPPING_CORE_SUBMAP_H_
#define PANOPTIC_MAPPING_CORE_SUBMAP_H_

#include <memory>
#include <string>

#include <voxblox/core/layer.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox_msgs/Mesh.h>

#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/core/submap_id.h"

namespace panoptic_mapping {

class Submap {
 public:
  struct Config {
    double voxel_size = 0.1;
    double voxels_per_side = 16;

    Config isValid() const;
  };

  explicit Submap(const Config& config);
  virtual ~Submap() = default;

  // io
  void getProto(SubmapProto* proto) const;
  bool saveToStream(std::fstream* outfile_ptr) const;
  static std::unique_ptr<Submap> loadFromStream(std::fstream* proto_file_ptr,
                                                uint64_t* tmp_byte_offset_ptr);

  // const accessors
  int getID() const { return id_; }
  int getInstanceID() const { return instance_id_; }
  const std::string& getFrameName() const { return frame_name_; }
  const Color& getColor() const { return color_; }
  const voxblox::Layer<TsdfVoxel>& getTsdfLayer() const { return *tsdf_layer_; }
  const voxblox::MeshLayer& getMeshLayer() const { return *mesh_layer_; }
  const Transformation& getT_M_S() const { return T_M_S_; }
  const Transformation& getT_S_M() const { return T_M_S_inv_; }
  bool isActive() const { return is_active_; }

  // modifying accessors
  std::shared_ptr<voxblox::Layer<TsdfVoxel>>& getTsdfLayerPtr() {
    return tsdf_layer_;
  }
  std::shared_ptr<voxblox::MeshLayer>& getMeshLayerPtr() { return mesh_layer_; }

  // setters
  void setColor(const Color& color) { color_ = color; }
  void setT_M_S(const Transformation& T_M_S);
  void setInstanceID(int id) { instance_id_ = id; }

  // processing
  void finishActivePeriod();

 private:
  friend class SubmapCollection;

  const Config config_;

  // state
  const SubmapID id_;
  int instance_id_;
  bool is_active_;

  // transformation
  std::string frame_name_;
  voxblox::Transformation T_M_S_;  // transform mission to submap
  voxblox::Transformation T_M_S_inv_;

  // map
  std::shared_ptr<voxblox::Layer<TsdfVoxel>> tsdf_layer_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;

  // visualization
  voxblox::Color color_;    // color to use for visualizers
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_SUBMAP_H_
