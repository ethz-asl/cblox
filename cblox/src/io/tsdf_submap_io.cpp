#include "cblox/core/tsdf_submap.h"
#include <string>
#include "cblox/io/tsdf_submap_io.h"

namespace cblox {
namespace io {

template<>
bool LoadSubmapFromStream<cblox::TsdfSubmap>(
    std::fstream* proto_file_ptr,
    typename SubmapCollection<cblox::TsdfSubmap>::Ptr submap_collection_ptr,
    uint64_t* tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK(submap_collection_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);

  // Getting the header for this submap
  SubmapProto submap_proto;
  if (!voxblox::utils::readProtoMsgFromStream(
      proto_file_ptr, &submap_proto, tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not read tsdf sub map protobuf message.";
    return false;
  }

  // Getting the transformation
  Transformation T_M_S;
  QuatTransformationProto transformation_proto = submap_proto.transform();
  conversions::transformProtoToKindr(transformation_proto, &T_M_S);

  LOG(INFO) << "Submap id: " << submap_proto.id();
  Eigen::Vector3 t = T_M_S.getPosition();
  Quaternion q = T_M_S.getRotation();
  LOG(INFO) << "[ " << t.x() << ", " << t.y() << ", " << t.z() << ", " << q.w()
            << q.x() << ", " << q.y() << ", " << q.z() << " ]";

  // Creating a new submap to hold the data
  submap_collection_ptr->createNewSubmap(T_M_S, submap_proto.id());

  // Getting the tsdf blocks for this submap (the tsdf layer)
  LOG(INFO) << "Tsdf number of allocated blocks: "
            << submap_proto.num_blocks();
  if (!voxblox::io::LoadBlocksFromStream(
      submap_proto.num_blocks(),
      Layer<TsdfVoxel>::BlockMergingStrategy::kReplace, proto_file_ptr,
      submap_collection_ptr->getActiveTsdfMapPtr()->getTsdfLayerPtr(),
      tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not load the tsdf blocks from stream.";
    return false;
  }

  return true;
}

}  // namespace io
}  // namespace cblox
