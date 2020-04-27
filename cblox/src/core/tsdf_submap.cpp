#include "cblox/core/tsdf_submap.h"

#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {

void TsdfSubmap::finishSubmap() {
  // Empty implementation.
}

void TsdfSubmap::prepareForPublish() {
  // Empty implementation.
}

void TsdfSubmap::getProto(SubmapProto* proto) const {
  CHECK_NOTNULL(proto);
  // Getting the relevant data
  size_t num_tsdf_blocks =
      tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks();
  QuatTransformationProto* transformation_proto_ptr =
      new QuatTransformationProto();
  conversions::transformKindrToProto(T_M_S_, transformation_proto_ptr);
  // Filling out the description of the submap
  proto->set_id(submap_id_);
  proto->set_num_blocks(num_tsdf_blocks);
  proto->set_num_esdf_blocks(0);
  proto->set_allocated_transform(transformation_proto_ptr);
}

bool TsdfSubmap::saveToStream(std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  // Saving the TSDF submap header
  SubmapProto submap_proto;
  getProto(&submap_proto);
  if (!voxblox::utils::writeProtoMsgToStream(submap_proto, outfile_ptr)) {
    LOG(ERROR) << "Could not write tsdf sub map message.";
    outfile_ptr->close();
    return false;
  }
  // Saving the blocks
  constexpr bool kIncludeAllBlocks = true;
  const Layer<TsdfVoxel>& tsdf_layer = tsdf_map_->getTsdfLayer();
  if (!tsdf_layer.saveBlocksToStream(kIncludeAllBlocks,
                                     voxblox::BlockIndexList(), outfile_ptr)) {
    LOG(ERROR) << "Could not write sub map blocks to stream.";
    outfile_ptr->close();
    return false;
  }
  // Success
  return true;
}

TsdfSubmap::Ptr TsdfSubmap::LoadFromStream(const Config& config,
                                           std::fstream* proto_file_ptr,
                                           uint64_t* tmp_byte_offset_ptr) {
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
  Transformation T_M_S;
  QuatTransformationProto transformation_proto = submap_proto.transform();
  conversions::transformProtoToKindr(transformation_proto, &T_M_S);

  LOG(INFO) << "Submap id: " << submap_proto.id();
  Eigen::Vector3 t = T_M_S.getPosition();
  Quaternion q = T_M_S.getRotation();
  LOG(INFO) << "[ " << t.x() << ", " << t.y() << ", " << t.z() << ", " << q.w()
            << ", " << q.x() << ", " << q.y() << ", " << q.z() << " ]";

  // Creating a new submap to hold the data
  auto submap_ptr =
      std::make_shared<TsdfSubmap>(T_M_S, submap_proto.id(), config);

  // Getting the tsdf blocks for this submap (the tsdf layer)
  LOG(INFO) << "Tsdf number of allocated blocks: " << submap_proto.num_blocks();
  if (!voxblox::io::LoadBlocksFromStream(
          submap_proto.num_blocks(),
          Layer<TsdfVoxel>::BlockMergingStrategy::kReplace, proto_file_ptr,
          submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr(),
          tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not load the tsdf blocks from stream.";
    return nullptr;
  }

  return submap_ptr;
}

}  // namespace cblox
