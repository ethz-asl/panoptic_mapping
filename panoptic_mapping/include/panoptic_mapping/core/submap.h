#ifndef PANOPTIC_MAPPING_CORE_SUBMAP_H_
#define PANOPTIC_MAPPING_CORE_SUBMAP_H_

#include <memory>
#include <string>
#include <vector>

#include <voxblox/core/layer.h>
#include <voxblox/mesh/mesh_layer.h>

#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/core/submap_id.h"

namespace panoptic_mapping {

class Submap {
 public:
  struct Config {
    double voxel_size = 0.1;
    double voxels_per_side = 16;

    [[nodiscard]] Config isValid() const;
  };

  // Iso-surface-points are used to check alignment and represent the surface
  // of finished submaps.
  struct IsoSurfacePoint {
    Point position;
    FloatingPoint distance;
    FloatingPoint weight;
  };

  struct ChangeDetectionData {
    bool is_matched = false;
    int matching_submap_id = 0;
  };

  explicit Submap(const Config& config);
  virtual ~Submap() = default;

  // io
  void getProto(SubmapProto* proto) const;
  bool saveToStream(std::fstream* outfile_ptr) const;
  static std::unique_ptr<Submap> loadFromStream(std::fstream* proto_file_ptr,
                                                uint64_t* tmp_byte_offset_ptr);

  // const accessors
  [[nodiscard]] int getID() const {
    return id_.toInt();
  }[[nodiscard]] int getInstanceID() const {
    return instance_id_;
  }
  [[nodiscard]] int getClassID() const {
    return class_id_;
  }[[nodiscard]] const std::string& getFrameName() const {
    return frame_name_;
  }
  [[nodiscard]] const TsdfLayer& getTsdfLayer() const {
    return *tsdf_layer_;
  }[[nodiscard]] const voxblox::MeshLayer& getMeshLayer() const {
    return *mesh_layer_;
  }
  [[nodiscard]] const Transformation& getT_M_S() const {
    return T_M_S_;
  }[[nodiscard]] const Transformation& getT_S_M() const {
    return T_M_S_inv_;
  }
  [[nodiscard]] bool isActive() const { return is_active_; }[[nodiscard]] const
      std::vector<IsoSurfacePoint>& getIsoSurfacePoints() const {
    return iso_surface_points_;
  }
  [[nodiscard]] const ChangeDetectionData& getChangeDetectionData() const {
    return change_detection_data_;
  }

  // modifying accessors
  std::shared_ptr<TsdfLayer>& getTsdfLayerPtr() {
    return tsdf_layer_;
  }
  std::shared_ptr<voxblox::MeshLayer>& getMeshLayerPtr() { return mesh_layer_; }
  std::vector<IsoSurfacePoint>* getIsoSurfacePointsPtr() {
    return &iso_surface_points_;
  }
  ChangeDetectionData* getChangeDetectionDataPtr() {
    return &change_detection_data_;
  }

  // setters
  void setT_M_S(const Transformation& T_M_S);
  void setInstanceID(int id) { instance_id_ = id; }
  void setClassID(int id) { class_id_ = id; }
  void setFrameName(const std::string& name) { frame_name_ = name; }

  // processing
  void finishActivePeriod();

 private:
  friend class SubmapCollection;

  const Config config_;

  // state
  const SubmapID id_;
  int instance_id_;
  int class_id_;
  bool is_active_;

  // transformation
  std::string frame_name_;
  Transformation T_M_S_;  // Transformation mission to submap.
  Transformation T_M_S_inv_;

  // map
  std::shared_ptr<TsdfLayer> tsdf_layer_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
  std::vector<IsoSurfacePoint> iso_surface_points_;
  ChangeDetectionData change_detection_data_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_SUBMAP_H_
