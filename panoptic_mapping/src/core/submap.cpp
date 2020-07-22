#include "panoptic_mapping/core/submap.h"

#include <memory>

#include <cblox/QuatTransformation.pb.h>
#include <cblox/utils/quat_transformation_protobuf_utils.h>
#include <voxblox/io/layer_io.h>

namespace panoptic_mapping {

Submap::Config Submap::Config::isValid() const {
  CHECK_GT(voxel_size, 0.0) << "The voxel size is expected > 0.0.";
  CHECK_GT(voxels_per_side, 0.0) << "The voxels per side are expected > 0.";
  return Config(*this);
}

Submap::Submap(const Config& config)
    : config_(config.isValid()),
      frame_name_("world"),
      instance_id_(-1),
      is_active_(true) {
  // setup layers
  tsdf_layer_ = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(
      config_.voxel_size, config_.voxels_per_side);
  double block_size =
      config_.voxel_size * static_cast<double>(config_.voxels_per_side);
  mesh_layer_ = std::make_shared<voxblox::MeshLayer>(block_size);

  // initialize with identity transformation
  Eigen::Quaterniond q(1, 0, 0, 0);
  Eigen::Vector3d t(0, 0, 0);
  T_M_S_ = Transformation(q.cast<voxblox::FloatingPoint>(),
                          t.cast<voxblox::FloatingPoint>());
  T_M_S_inv_ = T_M_S_;
}

void Submap::setT_M_S(const Transformation& T_M_S) {
  T_M_S_ = T_M_S;
  T_M_S_inv_ = T_M_S_.inverse();
}

void Submap::finishActivePeriod() { is_active_ = false; }

void Submap::getProto(SubmapProto* proto) const {
  CHECK_NOTNULL(proto);
  // Getting the relevant data
  size_t num_tsdf_blocks = tsdf_layer_->getNumberOfAllocatedBlocks();
  auto transformation_proto_ptr = new cblox::QuatTransformationProto();
  cblox::conversions::transformKindrToProto(T_M_S_, transformation_proto_ptr);

  // Filling out the description of the submap
  proto->set_instance_id(instance_id_);
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
  const Layer<TsdfVoxel>& tsdf_layer = *tsdf_layer_;
  if (!tsdf_layer.saveBlocksToStream(kIncludeAllBlocks,
                                     voxblox::BlockIndexList(), outfile_ptr)) {
    LOG(ERROR) << "Could not write submap blocks to stream.";
    outfile_ptr->close();
    return false;
  }
  return true;
}

std::unique_ptr<Submap> Submap::loadFromStream(std::fstream* proto_file_ptr,
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
          submap_proto.num_blocks(),
          Layer<TsdfVoxel>::BlockMergingStrategy::kReplace, proto_file_ptr,
          submap->tsdf_layer_.get(), tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not load the tsdf blocks from stream.";
    return nullptr;
  }

  // Getting the transformation
  Transformation T_M_S;
  cblox::QuatTransformationProto transformation_proto =
      submap_proto.transform();
  cblox::conversions::transformProtoToKindr(transformation_proto, &T_M_S);
  submap->setT_M_S(T_M_S);

  // other data
  submap->setInstanceID(submap_proto.instance_id());

  // recompute required data
  submap->finishActivePeriod();
  return submap;
}

}  // namespace panoptic_mapping
