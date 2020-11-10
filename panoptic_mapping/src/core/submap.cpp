#include "panoptic_mapping/core/submap.h"

#include <memory>
#include <sstream>

#include <cblox/QuatTransformation.pb.h>
#include <cblox/utils/quat_transformation_protobuf_utils.h>
#include <voxblox/io/layer_io.h>

namespace panoptic_mapping {

void Submap::Config::checkParams() const {
  checkParamGT(voxel_size, 0.0, "voxel_size");
  checkParamGT(voxels_per_side, 0.0, "voxels_per_side");
}

void Submap::Config::setupParamsAndPrinting() {
  setupParam("voxel_size", &voxel_size);
  setupParam("voxels_per_side", &voxels_per_side);
}

Submap::Submap(const Config& config)
    : config_(config.checkValid()),
      instance_id_(-1),
      class_id_(-1),
      is_active_(true),
      label_(PanopticLabel::kUNKNOWN),
      bounding_volume_(*this) {
  // Default values.
  std::stringstream ss;
  ss << "submap_" << id_.toInt();
  frame_name_ = ss.str();

  // Setup layers.
  tsdf_layer_ = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(
      config_.voxel_size, config_.voxels_per_side);
  double block_size =
      config_.voxel_size * static_cast<double>(config_.voxels_per_side);
  mesh_layer_ = std::make_shared<voxblox::MeshLayer>(block_size);

  // Initialize with identity transformation.
  T_M_S_.setIdentity();
  T_M_S_inv_.setIdentity();
}

void Submap::setT_M_S(const Transformation& T_M_S) {
  T_M_S_ = T_M_S;
  T_M_S_inv_ = T_M_S_.inverse();
}

void Submap::setLabel(const PanopticLabel& label) { label_ = label; }

void Submap::finishActivePeriod() { is_active_ = false; }

void Submap::getProto(SubmapProto* proto) const {
  CHECK_NOTNULL(proto);
  // Getting the relevant data
  size_t num_tsdf_blocks = tsdf_layer_->getNumberOfAllocatedBlocks();
  auto transformation_proto_ptr = new cblox::QuatTransformationProto();
  cblox::conversions::transformKindrToProto(T_M_S_, transformation_proto_ptr);

  // Filling out the description of the submap
  proto->set_instance_id(instance_id_);
  proto->set_class_id(class_id_);
  proto->set_panoptic_label(static_cast<int>(label_));
  proto->set_num_blocks(num_tsdf_blocks);
  proto->set_voxel_size(tsdf_layer_->voxel_size());
  proto->set_voxels_per_side(tsdf_layer_->voxels_per_side());
  proto->set_allocated_transform(transformation_proto_ptr);
}

bool Submap::saveToStream(std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  // Saving the TSDF submap header
  SubmapProto submap_proto;
  getProto(&submap_proto);
  if (!voxblox::utils::writeProtoMsgToStream(submap_proto, outfile_ptr)) {
    LOG(ERROR) << "Could not write submap proto message.";
    outfile_ptr->close();
    return false;
  }

  // Saving the blocks
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

  // Getting the header for this submap
  SubmapProto submap_proto;
  if (!voxblox::utils::readProtoMsgFromStream(proto_file_ptr, &submap_proto,
                                              tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not read tsdf submap protobuf message.";
    return nullptr;
  }

  // Creating a new submap to hold the data
  Config cfg;
  cfg.voxel_size = submap_proto.voxel_size();
  cfg.voxels_per_side = submap_proto.voxels_per_side();
  auto submap = std::make_unique<Submap>(cfg);

  // Getting the tsdf blocks for this submap (the tsdf layer)
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

  // recompute required data
  submap->finishActivePeriod();
  return submap;
}

}  // namespace panoptic_mapping
