#include "cblox/core/tsdf_submap.hpp"
#include "cblox/utils/quat_transformation_protobuf_utils.hpp"

namespace cblox {

void TsdfSubmap::getProto(TsdfSubmapProto* proto) const {
  // Checks
  CHECK_NOTNULL(proto);
  // Getting the relevant data
  size_t num_blocks = tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks();
  QuatTransformationProto* transformation_proto_ptr =
      new QuatTransformationProto();
  conversions::transformKindrToProto(T_M_S_, transformation_proto_ptr);
  // Filling out the description of the submap
  proto->set_keyframe_id(keyframe_id_);
  proto->set_num_blocks(num_blocks);
  proto->set_allocated_transform(transformation_proto_ptr);
}

bool TsdfSubmap::saveToStream(std::fstream* outfile_ptr) const {
  // Checks
  CHECK_NOTNULL(outfile_ptr);
  // Saving the TSDF submap header
  TsdfSubmapProto tsdf_sub_map_proto;
  getProto(&tsdf_sub_map_proto);
  if (!utils::writeProtoMsgToStream(tsdf_sub_map_proto, outfile_ptr)) {
    LOG(ERROR) << "Could not write tsdf sub map message.";
    outfile_ptr->close();
    return false;
  }
  // Saving the blocks
  constexpr bool kIncludeAllBlocks = true;
  const Layer<TsdfVoxel>& tsdf_layer = tsdf_map_->getTsdfLayer();
  if (!tsdf_layer.saveBlocksToStream(kIncludeAllBlocks, BlockIndexList(),
                                     outfile_ptr)) {
    LOG(ERROR) << "Could not write sub map blocks to stream.";
    outfile_ptr->close();
    return false;
  }
  // Success
  return true;
}

} // namespace cblox
