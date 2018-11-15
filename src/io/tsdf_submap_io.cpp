#include <iostream>
#include <string>

#include <glog/logging.h>

#include <voxblox/io/layer_io.h>

#include "./QuatTransformation.pb.h"
#include "./TsdfSubmap.pb.h"
#include "./TsdfSubmapCollection.pb.h"

#include "cblox/core/tsdf_submap.hpp"
#include "cblox/io/tsdf_submap_io.hpp"
#include "cblox/utils/quat_transformation_protobuf_utils.hpp"

namespace cblox {
namespace io {

bool SaveTsdfsubmapCollection(
    const TsdfSubmapCollection &tsdf_submap_collection,
    const std::string &file_path) {
  return tsdf_submap_collection.saveToFile(file_path);
}

bool LoadTsdfSubmapFromStream(
    std::fstream *proto_file_ptr,
    TsdfSubmapCollection::Ptr tsdf_submap_collection_ptr,
    uint32_t *tmp_byte_offset_ptr) {
  // Checks
  CHECK_NOTNULL(proto_file_ptr);
  CHECK(tsdf_submap_collection_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);

  // Getting the header for this submap
  TsdfSubmapProto tsdf_sub_map_proto;
  if (!utils::readProtoMsgFromStream(proto_file_ptr, &tsdf_sub_map_proto,
                                     tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not read tsdf sub map protobuf message.";
    return false;
  }

  // Getting the transformation
  Transformation T_M_S;
  QuatTransformationProto transformation_proto = tsdf_sub_map_proto.transform();
  conversions::transformProtoToKindr(transformation_proto, &T_M_S);

  // DEBUG
  std::cout << "Tsdf keyframe id: " << tsdf_sub_map_proto.keyframe_id()
            << std::endl;
  std::cout << "Tsdf number of allocated blocks: "
            << tsdf_sub_map_proto.num_blocks() << std::endl;
  Eigen::Vector3 t = T_M_S.getPosition();
  Quaternion q = T_M_S.getRotation();
  std::cout << "[ " << t.x() << ", " << t.y() << ", " << t.z() << ", " << q.w()
            << q.x() << ", " << q.y() << ", " << q.z() << " ]" << std::endl;

  // Creating a new submap to hold the data
  tsdf_submap_collection_ptr->createNewSubMap(T_M_S,
                                              tsdf_sub_map_proto.keyframe_id());

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

bool LoadTsdfSubmapCollection(
    const std::string &file_path,
    TsdfSubmapCollection::Ptr *tsdf_submap_collection_ptr) {
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
  if (!utils::readProtoMsgFromStream(&proto_file, &tsdf_submap_collection_proto,
                                     &tmp_byte_offset)) {
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
  TsdfMap::Config tsdf_map_config;
  tsdf_map_config.tsdf_voxel_size = tsdf_submap_collection_proto.voxel_size();
  tsdf_map_config.tsdf_voxels_per_side =
      tsdf_submap_collection_proto.voxels_per_side();
  tsdf_submap_collection_ptr->reset(new TsdfSubmapCollection(tsdf_map_config));

  // Loading each of the tsdf sub maps
  for (size_t sub_map_index = 0;
       sub_map_index < tsdf_submap_collection_proto.num_submaps();
       sub_map_index++) {
    // DEBUG
    std::cout << "Loading tsdf sub map number: " << sub_map_index << std::endl;
    // Loading the submaps
    if (!LoadTsdfSubmapFromStream(&proto_file, *tsdf_submap_collection_ptr,
                                  &tmp_byte_offset)) {
      LOG(ERROR) << "Could not load the tsdf sub map from stream.";
      return false;
    }
  }
  // Because grown ups clean up after themselves
  proto_file.close();
  return true;
}

bool SaveTransformationArray(
    const AlignedVector<Transformation> &transformation_array,
    const std::string &file_path) {
  // Opening the file (if we can)
  CHECK(!file_path.empty());
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::out | std::fstream::binary);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << file_path;
    return false;
  }

  // Write the total number of messages to the beginning of this file.
  const uint32_t num_messages = transformation_array.size();
  if (!utils::writeProtoMsgCountToStream(num_messages, &proto_file)) {
    LOG(ERROR) << "Could not write message number to file.";
    proto_file.close();
    return false;
  }

  // Looping over the transforms and saving them to an array
  size_t transform_index = 0;
  for (const Transformation &transform : transformation_array) {
    // DEUBG
    std::cout << "Saving the transform number: " << transform_index
              << std::endl;
    transform_index++;

    // Getting the proto
    QuatTransformationProto transformation_proto;
    conversions::transformKindrToProto(transform, &transformation_proto);
    // Saving to the stream
    if (!utils::writeProtoMsgToStream(transformation_proto, &proto_file)) {
      LOG(ERROR) << "Could not write transform message.";
      proto_file.close();
      return false;
    }
  }

  // Because grown ups clean up after themselves
  proto_file.close();
  return true;
}

bool LoadTransformationArray(
    const std::string &file_path,
    AlignedVector<Transformation> *transformation_array_ptr) {
  // Checks
  CHECK_NOTNULL(transformation_array_ptr);

  // Opening the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }

  // Unused byte offset result.
  uint32_t tmp_byte_offset = 0u;

  // Get number of messages
  uint32_t num_protos;
  if (!utils::readProtoMsgCountToStream(&proto_file, &num_protos,
                                        &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read number of messages.";
    return false;
  }

  if (num_protos == 0u) {
    LOG(WARNING) << "Empty protobuf file!";
    return false;
  }

  // DEBUG
  std::cout << "num_protos: " << num_protos << std::endl;

  // Reading all the transforms
  transformation_array_ptr->clear();
  transformation_array_ptr->reserve(num_protos);
  for (size_t transform_index = 0; transform_index < num_protos;
       transform_index++) {
    // DEBUG
    std::cout << "transform_index: " << transform_index << std::endl;

    // Getting the transformation proto
    QuatTransformationProto transformation_proto;
    if (!utils::readProtoMsgFromStream(&proto_file, &transformation_proto,
                                       &tmp_byte_offset)) {
      LOG(ERROR) << "Could not read transform protobuf message.";
      return false;
    }
    // Converting to minkindr
    Transformation T_M_S;
    conversions::transformProtoToKindr(transformation_proto, &T_M_S);
    // Adding to the transformation to the array
    transformation_array_ptr->push_back(T_M_S);
  }

  // Because grown ups clean up after themselves
  proto_file.close();
  return true;
}

}  // namespace io

} // namespace cblox
