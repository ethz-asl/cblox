#ifndef CBLOX_IO_TSDF_SUBMAP_IO_INL_H_
#define CBLOX_IO_TSDF_SUBMAP_IO_INL_H_

#include <string>

#include <glog/logging.h>

#include <voxblox/io/layer_io.h>

#include "cblox/QuatTransformation.pb.h"
#include "cblox/Submap.pb.h"
#include "cblox/SubmapCollection.pb.h"
#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace io {

// We assume all SubmapTypes to have a TSDF map.
template <typename SubmapType>
bool LoadSubmapFromStream(
    std::fstream* proto_file_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr,
    uint64_t* tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK(submap_collection_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);

  // Getting the header for this submap
  SubmapProto submap_proto;
  if (!voxblox::utils::readProtoMsgFromStream(proto_file_ptr, &submap_proto,
                                              tmp_byte_offset_ptr)) {
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
            << ", " << q.x() << ", " << q.y() << ", " << q.z() << " ]";

  // Creating a new submap to hold the data
  submap_collection_ptr->createNewSubmap(T_M_S, submap_proto.id());

  // Getting the tsdf blocks for this submap (the tsdf layer)
  LOG(INFO) << "Tsdf number of allocated blocks: " << submap_proto.num_blocks();
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

template <typename SubmapType>
bool SaveSubmapCollection(const SubmapCollection<SubmapType>& submap_collection,
                          const std::string& file_path) {
  return submap_collection.saveToFile(file_path);
}

template <typename SubmapType>
bool LoadSubmapCollection(
    const std::string& file_path,
    typename SubmapCollection<SubmapType>::Ptr* submap_collection_ptr) {
  CHECK_NOTNULL(submap_collection_ptr);
  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }
  // Unused byte offset result.
  uint64_t tmp_byte_offset = 0;
  // Loading the header
  SubmapCollectionProto submap_collection_proto;
  if (!voxblox::utils::readProtoMsgFromStream(
          &proto_file, &submap_collection_proto, &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read tsdf submap collection map protobuf message.";
    return false;
  }

  LOG(INFO) << "tsdf_submap_collection_proto.num_submaps(): "
            << submap_collection_proto.num_submaps();

  // Loading each of the submaps
  for (size_t sub_map_index = 0;
       sub_map_index < submap_collection_proto.num_submaps(); sub_map_index++) {
    LOG(INFO) << "Loading submap number: " << sub_map_index;
    // Loading the submaps
    if (!LoadSubmapFromStream<SubmapType>(&proto_file, *submap_collection_ptr,
                                          &tmp_byte_offset)) {
      LOG(ERROR) << "Could not load the submap from stream.";
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
