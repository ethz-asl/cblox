#include "cblox/core/tsdf_submap.h"

#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {

void TsdfSubmap::getProto(SubmapProto* proto) const {
  CHECK_NOTNULL(proto);
  // Getting the relevant data
  size_t num_tsdf_blocks = tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks();
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

}  // namespace cblox
