#include "panoptic_mapping/core/submap.h"

#include <voxblox/io/layer_io.h>
#include <memory>

namespace panoptic_mapping {

Submap::Submap(int id, double voxel_size, int voxels_per_side) : id_(id),
                                                                 frame_name_("world") {
  // setup layers
  tsdf_layer_ = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(voxel_size, voxels_per_side);
  double block_size = voxel_size * (double) voxels_per_side;
  mesh_layer_ = std::make_shared<voxblox::MeshLayer>(block_size);

  // initialize with identity transformation
  Eigen::Quaterniond q(1, 0, 0, 0);
  Eigen::Vector3d t(0, 0, 0);
  T_M_S_ = Transformation(q.cast<voxblox::FloatingPoint>(), t.cast<voxblox::FloatingPoint>());
  T_M_S_inv_ = T_M_S_;
}

void Submap::setT_M_S(const Transformation &T_M_S) {
  T_M_S_ = T_M_S;
  T_M_S_inv_ = T_M_S_.inverse();
}

void Submap::getProto(SubmapProto *proto) const {
  CHECK_NOTNULL(proto);
  // Getting the relevant data
  size_t num_tsdf_blocks = tsdf_layer_->getNumberOfAllocatedBlocks();
  //QuatTransformationProto* transformation_proto_ptr = new QuatTransformationProto();
  //conversions::transformKindrToProto(T_M_S_, transformation_proto_ptr);
  // Filling out the description of the submap
  proto->set_id(id_);
  proto->set_num_blocks(num_tsdf_blocks);
  proto->set_voxel_size(tsdf_layer_->voxel_size());
  proto->set_voxels_per_side(tsdf_layer_->voxels_per_side());
  //proto->set_allocated_transform(transformation_proto_ptr);
}

bool Submap::saveToStream(std::fstream *outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  // Saving the TSDF submap header
  SubmapProto submap_proto;
  getProto(&submap_proto);
  if (!voxblox::utils::writeProtoMsgToStream(submap_proto, outfile_ptr)) {
    LOG(ERROR) << "Could not write submap message.";
    outfile_ptr->close();
    return false;
  }
  // Saving the blocks
  constexpr bool kIncludeAllBlocks = true;
  const Layer<TsdfVoxel> &tsdf_layer = *tsdf_layer_;
  if (!tsdf_layer.saveBlocksToStream(kIncludeAllBlocks,
                                     voxblox::BlockIndexList(), outfile_ptr)) {
    LOG(ERROR) << "Could not write sub map blocks to stream.";
    outfile_ptr->close();
    return false;
  }
  return true;
}

std::unique_ptr<Submap> Submap::loadFromStream(std::fstream *proto_file_ptr, uint64_t *tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);

  // Getting the header for this submap
  SubmapProto submap_proto;
  if (!voxblox::utils::readProtoMsgFromStream(proto_file_ptr, &submap_proto,
                                              tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not read tsdf sub map protobuf message.";
    return nullptr;
  }

  // Getting the transformation
//  Transformation T_M_S;
//  QuatTransformationProto transformation_proto = submap_proto.transform();
//  conversions::transformProtoToKindr(transformation_proto, &T_M_S);
//
//  LOG(INFO) << "Submap id: " << submap_proto.id();
//  Eigen::Vector3 t = T_M_S.getPosition();
//  Quaternion q = T_M_S.getRotation();
//  LOG(INFO) << "[ " << t.x() << ", " << t.y() << ", " << t.z() << ", " << q.w()
//            << ", " << q.x() << ", " << q.y() << ", " << q.z() << " ]";

  // Creating a new submap to hold the data
  auto submap =
      std::unique_ptr<Submap>(new Submap(submap_proto.id(), submap_proto.voxel_size(), submap_proto.voxels_per_side()));

  // Getting the tsdf blocks for this submap (the tsdf layer)
  LOG(INFO) << "Tsdf number of allocated blocks: " << submap_proto.num_blocks();
  if (!voxblox::io::LoadBlocksFromStream(submap_proto.num_blocks(),
                                         Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
                                         proto_file_ptr,
                                         submap->tsdf_layer_.get(),
                                         tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not load the tsdf blocks from stream.";
    return nullptr;
  }
  return submap;
}

} // namespace panoptic_mapping
