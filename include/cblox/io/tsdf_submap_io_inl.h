//
// Created by victor on 14.01.19.
//

#ifndef CBLOX_IO_TSDF_SUBMAP_IO_INL_H_
#define CBLOX_IO_TSDF_SUBMAP_IO_INL_H_

#include <voxblox/io/layer_io.h>
#include <string>

#include "./QuatTransformation.pb.h"
#include "./TsdfSubmap.pb.h"
#include "./TsdfSubmapCollection.pb.h"

#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace io {

template <typename SubmapType>
bool LoadSubmapFromStream(
    std::fstream *proto_file_ptr,
    typename SubmapCollection<SubmapType>::Ptr tsdf_submap_collection_ptr,
    uint32_t *tmp_byte_offset_ptr) {
  // Checks
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

  // DEBUG
  std::cout << "Tsdf submap id: " << tsdf_sub_map_proto.id() << std::endl;
  std::cout << "Tsdf number of allocated blocks: "
            << tsdf_sub_map_proto.num_blocks() << std::endl;
  Eigen::Vector3 t = T_M_S.getPosition();
  Quaternion q = T_M_S.getRotation();
  std::cout << "[ " << t.x() << ", " << t.y() << ", " << t.z() << ", " << q.w()
            << q.x() << ", " << q.y() << ", " << q.z() << " ]" << std::endl;

  // Creating a new submap to hold the data
  tsdf_submap_collection_ptr->createNewSubMap(T_M_S, tsdf_sub_map_proto.id());

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
    const std::string &file_path,
    typename SubmapCollection<SubmapType>::Ptr *tsdf_submap_collection_ptr) {
  // Checks
  // CHECK(tsdf_submap_collection_ptr);
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
  // DEBUG
  std::cout << "tsdf_submap_collection_proto.voxel_size(): "
            << tsdf_submap_collection_proto.voxel_size() << std::endl;
  std::cout << "tsdf_submap_collection_proto.voxels_per_side(): "
            << tsdf_submap_collection_proto.voxels_per_side() << std::endl;
  std::cout << "tsdf_submap_collection_proto.num_submaps(): "
            << tsdf_submap_collection_proto.num_submaps() << std::endl;

  // Creating the new submap collection based on the loaded parameters
  typename SubmapType::Config tsdf_map_config;
  tsdf_map_config.tsdf_voxel_size = tsdf_submap_collection_proto.voxel_size();
  tsdf_map_config.tsdf_voxels_per_side =
      tsdf_submap_collection_proto.voxels_per_side();
  tsdf_submap_collection_ptr->reset(
      new SubmapCollection<SubmapType>(tsdf_map_config));

  // Loading each of the tsdf sub maps
  for (size_t sub_map_index = 0;
       sub_map_index < tsdf_submap_collection_proto.num_submaps();
       sub_map_index++) {
    // DEBUG
    std::cout << "Loading tsdf sub map number: " << sub_map_index << std::endl;
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
