#ifndef CBLOX_IO_TSDF_SUBMAP_IO_INL_H_
#define CBLOX_IO_TSDF_SUBMAP_IO_INL_H_

#include <string>

#include <voxblox/io/layer_io.h>

#include <glog/logging.h>

#include "./QuatTransformation.pb.h"
#include "./TsdfSubmap.pb.h"
#include "./TsdfSubmapCollection.pb.h"

#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace io {

template <typename SubmapType>
bool LoadSubmapFromStream(
    std::fstream* proto_file_ptr,
    typename SubmapCollection<SubmapType>::Ptr tsdf_submap_collection_ptr,
    uint32_t* tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK(tsdf_submap_collection_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);

  // Getting the header for this submap
  TsdfSubmapProto tsdf_sub_map_proto;
  if (!voxblox::utils::readProtoMsgFromStream(
          proto_file_ptr, &tsdf_sub_map_proto, tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not read tsdf sub map protobuf message.";
    return false;
  }

  // Getting the transformation
  Transformation T_M_S;
  QuatTransformationProto transformation_proto = tsdf_sub_map_proto.transform();
  conversions::transformProtoToKindr(transformation_proto, &T_M_S);

  LOG(INFO) << "Tsdf submap id: " << tsdf_sub_map_proto.id();
  LOG(INFO) << "Tsdf number of allocated blocks: "
            << tsdf_sub_map_proto.num_blocks();
  Eigen::Vector3 t = T_M_S.getPosition();
  Quaternion q = T_M_S.getRotation();
  LOG(INFO) << "[ " << t.x() << ", " << t.y() << ", " << t.z() << ", " << q.w()
            << q.x() << ", " << q.y() << ", " << q.z() << " ]";

  // Creating a new submap to hold the data
  tsdf_submap_collection_ptr->createNewSubmap(T_M_S, tsdf_sub_map_proto.id());

  // Getting the blocks for this submap (the tsdf layer)
  if (!voxblox::io::LoadBlocksFromStream(
          tsdf_sub_map_proto.num_blocks(),
          Layer<TsdfVoxel>::BlockMergingStrategy::kReplace, proto_file_ptr,
          tsdf_submap_collection_ptr->getActiveTsdfMapPtr()->getTsdfLayerPtr(),
          tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not load the blocks from stream.";
    return false;
  }

  return true;
}

template <typename SubmapType>
bool LoadSubmapCollection(
    const std::string& file_path,
    typename SubmapCollection<SubmapType>::Ptr* tsdf_submap_collection_ptr) {
  CHECK_NOTNULL(tsdf_submap_collection_ptr);
  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }
  // Unused byte offset result.
  uint32_t tmp_byte_offset = 0;
  // Loading the header
  TsdfSubmapCollectionProto tsdf_submap_collection_proto;
  if (!voxblox::utils::readProtoMsgFromStream(
          &proto_file, &tsdf_submap_collection_proto, &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read tsdf submap collection map protobuf message.";
    return false;
  }

  LOG(INFO) << "tsdf_submap_collection_proto.num_submaps(): "
            << tsdf_submap_collection_proto.num_submaps();

  // Loading each of the tsdf sub maps
  for (size_t sub_map_index = 0;
       sub_map_index < tsdf_submap_collection_proto.num_submaps();
       sub_map_index++) {
    LOG(INFO) << "Loading tsdf sub map number: " << sub_map_index;
    // Loading the submaps
    if (!LoadSubmapFromStream<SubmapType>(
            &proto_file, *tsdf_submap_collection_ptr, &tmp_byte_offset)) {
      LOG(ERROR) << "Could not load the tsdf sub map from stream.";
      return false;
    }
  }
  // Because grown ups clean up after themselves
  proto_file.close();
  return true;
}

}  // namespace io
}  // namespace cblox

#endif  // CBLOX_IO_TSDF_SUBMAP_IO_INL_H_
