#include "panoptic_mapping/map/submap.h"

#include <memory>
#include <sstream>
#include <vector>

#include <cblox/QuatTransformation.pb.h>
#include <cblox/utils/quat_transformation_protobuf_utils.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace panoptic_mapping {

void Submap::Config::checkParams() const {
  checkParamGT(voxel_size, 0.f, "voxel_size");
  checkParamGT(truncation_distance, 0.f, "truncation_distance");
  checkParamGT(voxels_per_side, 0, "voxels_per_side");
  checkParamConfig(mesh_config);
}

void Submap::Config::setupParamsAndPrinting() {
  setupParam("voxel_size", &voxel_size);
  setupParam("truncation_distance", &truncation_distance);
  setupParam("voxels_per_side", &voxels_per_side);
  setupParam("mesh_config", &mesh_config);
  setupParam("use_class_layer", &use_class_layer);
}

Submap::Submap(const Config& config)
    : config_(config.checkValid()), bounding_volume_(*this) {
  // Default values.
  std::stringstream ss;
  ss << "submap_" << static_cast<int>(id_);
  frame_name_ = ss.str();

  // Initialize with identity transformation.
  T_M_S_.setIdentity();
  T_M_S_inv_.setIdentity();

  // Setup layers.
  tsdf_layer_ = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(
      config_.voxel_size, config_.voxels_per_side);
  mesh_layer_ = std::make_shared<voxblox::MeshLayer>(config_.voxel_size *
                                                     config_.voxels_per_side);
  if (config_.use_class_layer) {
    class_layer_ = std::make_shared<voxblox::Layer<ClassVoxel>>(
        config_.voxel_size, config_.voxels_per_side);
  }

  // Setup tools.
  if (config_.use_class_layer) {
    mesh_integrator_ = std::make_unique<MeshIntegrator>(
        config_.mesh_config, tsdf_layer_, mesh_layer_, class_layer_,
        config_.truncation_distance);
  } else {
    mesh_integrator_ = std::make_unique<MeshIntegrator>(
        config_.mesh_config, tsdf_layer_, mesh_layer_);
  }
}

void Submap::setT_M_S(const Transformation& T_M_S) {
  T_M_S_ = T_M_S;
  T_M_S_inv_ = T_M_S_.inverse();
}

void Submap::finishActivePeriod() {
  // TODO(schmluk): at the moment these things are specifically set when loading
  // submaps, need to update automatically once submaps can go out of scope.
  is_active_ = false;
  change_state_ = ChangeState::kUnobserved;
  bounding_volume_.update();
}

void Submap::getProto(SubmapProto* proto) const {
  CHECK_NOTNULL(proto);
  // Getting the relevant data
  size_t num_tsdf_blocks = tsdf_layer_->getNumberOfAllocatedBlocks();
  auto transformation_proto_ptr = new cblox::QuatTransformationProto();
  cblox::conversions::transformKindrToProto(T_M_S_, transformation_proto_ptr);

  // Filling out the description of the submap.
  proto->set_instance_id(instance_id_);
  proto->set_class_id(class_id_);
  proto->set_panoptic_label(static_cast<int>(label_));
  proto->set_num_blocks(num_tsdf_blocks);
  proto->set_voxel_size(config_.voxel_size);
  proto->set_voxels_per_side(config_.voxels_per_side);
  proto->set_truncation_distance(config_.truncation_distance);
  proto->set_allocated_transform(transformation_proto_ptr);
}

bool Submap::saveToStream(std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  // Saving the TSDF submap header.
  SubmapProto submap_proto;
  getProto(&submap_proto);
  if (!voxblox::utils::writeProtoMsgToStream(submap_proto, outfile_ptr)) {
    LOG(ERROR) << "Could not write submap proto message.";
    outfile_ptr->close();
    return false;
  }

  // Saving the blocks.
  constexpr bool kIncludeAllBlocks = true;
  const TsdfLayer& tsdf_layer = *tsdf_layer_;
  if (!tsdf_layer.saveBlocksToStream(kIncludeAllBlocks,
                                     voxblox::BlockIndexList(), outfile_ptr)) {
    LOG(ERROR) << "Could not write submap blocks to stream.";
    outfile_ptr->close();
    return false;
  }
  return true;
}

std::unique_ptr<Submap> Submap::loadFromStream(std::istream* proto_file_ptr,
                                               uint64_t* tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);

  // Getting the header for this submap.
  SubmapProto submap_proto;
  if (!voxblox::utils::readProtoMsgFromStream(proto_file_ptr, &submap_proto,
                                              tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not read tsdf submap protobuf message.";
    return nullptr;
  }

  // Creating a new submap to hold the data.
  Config cfg;
  cfg.voxel_size = submap_proto.voxel_size();
  cfg.voxels_per_side = submap_proto.voxels_per_side();
  cfg.truncation_distance = submap_proto.truncation_distance();
  auto submap = std::make_unique<Submap>(cfg);

  // Getting the tsdf blocks for this submap (the tsdf layer).
  if (!voxblox::io::LoadBlocksFromStream(
          submap_proto.num_blocks(), TsdfLayer::BlockMergingStrategy::kReplace,
          proto_file_ptr, submap->tsdf_layer_.get(), tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not load the tsdf blocks from stream.";
    return nullptr;
  }

  // Getting the transformation.
  Transformation T_M_S;
  cblox::QuatTransformationProto transformation_proto =
      submap_proto.transform();
  cblox::conversions::transformProtoToKindr(transformation_proto, &T_M_S);
  submap->setT_M_S(T_M_S);

  // Other data.
  submap->setInstanceID(submap_proto.instance_id());
  submap->setClassID(submap_proto.class_id());
  submap->setLabel(static_cast<PanopticLabel>(submap_proto.panoptic_label()));

  return submap;
}

void Submap::updateMesh(bool only_updated_blocks) {
  // Use the default integrator config to have color always available.
  mesh_integrator_->generateMesh(only_updated_blocks, true);
}

void Submap::computeIsoSurfacePoints() {
  // NOTE(schmluk): Currently all surface points are computed from scratch every
  // time, but since they are currently only computed when a submap is finished
  // it should be fine.
  // This function utilizes the stored mesh so make sure updateMesh is called
  // earlier.
  iso_surface_points_ = std::vector<IsoSurfacePoint>();

  // Create an interpolator to interpolate the vertex weights from the TSDF.
  voxblox::Interpolator<TsdfVoxel> interpolator(tsdf_layer_.get());

  // Extract the vertices and verify.
  voxblox::BlockIndexList index_list;
  mesh_layer_->getAllAllocatedMeshes(&index_list);
  for (const voxblox::BlockIndex& index : index_list) {
    const Pointcloud& vertices = mesh_layer_->getMeshByIndex(index).vertices;
    iso_surface_points_.reserve(iso_surface_points_.size() + vertices.size());
    for (const Point& vertex : vertices) {
      // Try to interpolate the voxel weight and verify the distance.
      TsdfVoxel voxel;
      if (interpolator.getVoxel(vertex, &voxel, true)) {
        CHECK_LE(voxel.distance, 1e-2 * config_.voxel_size);
        iso_surface_points_.emplace_back(vertex, voxel.weight);
      }
    }
  }
}

void Submap::updateBoundingVolume() { bounding_volume_.update(); }

}  // namespace panoptic_mapping
