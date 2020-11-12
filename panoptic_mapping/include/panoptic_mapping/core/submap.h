#ifndef PANOPTIC_MAPPING_CORE_SUBMAP_H_
#define PANOPTIC_MAPPING_CORE_SUBMAP_H_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <voxblox/core/layer.h>
#include <voxblox/mesh/mesh_layer.h>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/core/submap_bounding_volume.h"
#include "panoptic_mapping/core/submap_id.h"

namespace panoptic_mapping {

// Iso-surface-points are used to check alignment and represent the surface
// of finished submaps.
struct IsoSurfacePoint {
  Point position;
  FloatingPoint distance;
  FloatingPoint weight;
};

// Change detection data stores relevant information for associating submaps.
struct ChangeDetectionData {
  enum class State {
    kNew = 0,
    kMatched,
    kUnobserved,
    kAbsent,
    kPersistent
  } state;
  int matched_submap_id;  // currently only allows for a single match.
};

class Submap {
 public:
  struct Config : public config_utilities::Config<Config> {
    double voxel_size = 0.1;
    double voxels_per_side = 16;

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
  int getID() const { return id_.toInt(); }
  int getInstanceID() const { return instance_id_; }
  int getClassID() const { return class_id_; }
  const std::string& getFrameName() const { return frame_name_; }
  const TsdfLayer& getTsdfLayer() const { return *tsdf_layer_; }
  const voxblox::MeshLayer& getMeshLayer() const { return *mesh_layer_; }
  const Transformation& getT_M_S() const { return T_M_S_; }
  const Transformation& getT_S_M() const { return T_M_S_inv_; }
  bool isActive() const { return is_active_; }
  const std::vector<IsoSurfacePoint>& getIsoSurfacePoints() const {
    return iso_surface_points_;
  }
  const ChangeDetectionData& getChangeDetectionData() const {
    return change_detection_data_;
  }
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
  ChangeDetectionData* getChangeDetectionDataPtr() {
    return &change_detection_data_;
  }

  // Setters.
  void setT_M_S(const Transformation& T_M_S);
  void setInstanceID(int id) { instance_id_ = id; }
  void setClassID(int id) { class_id_ = id; }
  void setFrameName(const std::string& name) { frame_name_ = name; }
  void setLabel(PanopticLabel label) { label_ = label; }

  // Processing.
  void finishActivePeriod();

 private:
  friend class SubmapCollection;
  const Config config_;

  // Labels.
  const SubmapID id_;
  int instance_id_ = -1;
  int class_id_ = -1;
  PanopticLabel label_ = PanopticLabel::kUnknown;

  // State.
  bool is_active_ = true;

  // Transformations.
  std::string frame_name_;
  Transformation T_M_S_;  // Transformation mission to submap.
  Transformation T_M_S_inv_;

  // Map.
  std::shared_ptr<TsdfLayer> tsdf_layer_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;

  // Data.
  ChangeDetectionData change_detection_data_;
  std::vector<IsoSurfacePoint> iso_surface_points_;
  SubmapBoundingVolume bounding_volume_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_CORE_SUBMAP_H_
