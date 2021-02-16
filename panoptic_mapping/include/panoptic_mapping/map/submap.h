#ifndef PANOPTIC_MAPPING_MAP_SUBMAP_H_
#define PANOPTIC_MAPPING_MAP_SUBMAP_H_

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <voxblox/core/layer.h>
#include <voxblox/mesh/mesh_layer.h>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/instance_id.h"
#include "panoptic_mapping/map/submap_bounding_volume.h"
#include "panoptic_mapping/map/submap_id.h"

namespace panoptic_mapping {

// Iso-surface-points are used to check alignment and represent the surface
// of finished submaps.
struct IsoSurfacePoint {
  IsoSurfacePoint(Point _position, FloatingPoint _weight)
      : position(std::move(_position)), weight(_weight) {}
  Point position;
  FloatingPoint weight;
};

// Change detection data stores relevant information for associating submaps.
enum class ChangeState {
  kNew = 0,
  kMatched,
  kUnobserved,
  kAbsent,
  kPersistent
};

class Submap {
 public:
  struct Config : public config_utilities::Config<Config> {
    float voxel_size = 0.1;
    float truncation_distance = 0.0;  // Defaults to 2x voxel size.
    int voxels_per_side = 16;  // Needs to be a multiple of 2.

    void initializeDependentVariableDefaults() override;

    Config() { setConfigName("Submap"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit Submap(const Config& config);
  virtual ~Submap() = default;

  // IO.
  void getProto(SubmapProto* proto) const;
  bool saveToStream(std::fstream* outfile_ptr) const;
  static std::unique_ptr<Submap> loadFromStream(std::istream* proto_file_ptr,
                                                uint64_t* tmp_byte_offset_ptr);

  // Const accessors.
  const Config& getConfig() const { return config_; }
  int getID() const { return id_; }
  int getInstanceID() const { return instance_id_; }
  int getClassID() const { return class_id_; }
  const std::string& getFrameName() const { return frame_name_; }
  const std::string& getName() const { return name_; }
  const TsdfLayer& getTsdfLayer() const { return *tsdf_layer_; }
  const voxblox::MeshLayer& getMeshLayer() const { return *mesh_layer_; }
  const Transformation& getT_M_S() const { return T_M_S_; }
  const Transformation& getT_S_M() const { return T_M_S_inv_; }
  bool isActive() const { return is_active_; }
  const std::vector<IsoSurfacePoint>& getIsoSurfacePoints() const {
    return iso_surface_points_;
  }
  ChangeState getChangeState() const { return change_state_; }
  PanopticLabel getLabel() const { return label_; }
  const SubmapBoundingVolume& getBoundingVolume() const {
    return bounding_volume_;
  }

  // Modifying accessors.
  std::shared_ptr<TsdfLayer>& getTsdfLayerPtr() {
    return tsdf_layer_;
  }
  std::shared_ptr<voxblox::MeshLayer>& getMeshLayerPtr() { return mesh_layer_; }
  std::vector<IsoSurfacePoint>* getIsoSurfacePointsPtr() {
    return &iso_surface_points_;
  }
  SubmapBoundingVolume* getBoundingVolumePtr() { return &bounding_volume_; }

  // Setters.
  void setT_M_S(const Transformation& T_M_S);
  void setInstanceID(int id) { instance_id_ = id; }
  void setClassID(int id) { class_id_ = id; }
  void setFrameName(const std::string& name) { frame_name_ = name; }
  void setLabel(PanopticLabel label) { label_ = label; }
  void setName(const std::string& name) { name_ = name; }
  void setChangeState(ChangeState state) { change_state_ = state; }

  // Processing.
  void finishActivePeriod();

 private:
  friend class SubmapCollection;
  const Config config_;

  // Labels.
  const SubmapID id_;  // UUID
  InstanceID instance_id_;  // Per default sets up a new unique ID.
  int class_id_ = -1;
  PanopticLabel label_ = PanopticLabel::kUnknown;
  std::string name_;

  // State.
  bool is_active_ = true;
  ChangeState change_state_ = ChangeState::kNew;

  // Transformations.
  std::string frame_name_;
  Transformation T_M_S_;  // Transformation mission to submap.
  Transformation T_M_S_inv_;

  // Map.
  std::shared_ptr<TsdfLayer> tsdf_layer_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
  std::vector<IsoSurfacePoint> iso_surface_points_;
  SubmapBoundingVolume bounding_volume_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_SUBMAP_H_
