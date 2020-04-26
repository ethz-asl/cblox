#ifndef CBLOX_IO_SUBMAP_IO_INL_H_
#define CBLOX_IO_SUBMAP_IO_INL_H_

#include <string>

#include <glog/logging.h>

#include <voxblox/io/layer_io.h>

#include "cblox/QuatTransformation.pb.h"
#include "cblox/Submap.pb.h"
#include "cblox/SubmapCollection.pb.h"
#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace io {

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
  uint64_t tmp_byte_offset = 0u;
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
  for (size_t sub_map_index = 0u;
       sub_map_index < submap_collection_proto.num_submaps(); ++sub_map_index) {
    LOG(INFO) << "Loading submap number: " << sub_map_index;
    // Loading the submaps
    typename SubmapType::Ptr submap_ptr;
    if (!SubmapType::LoadFromStream((*submap_collection_ptr)->getConfig(),
                                    &proto_file, &tmp_byte_offset,
                                    &submap_ptr)) {
      LOG(ERROR) << "Could not load the submap from stream.";
      return false;
    }
    (*submap_collection_ptr)->addSubmap(submap_ptr);
  }
  // Because grown ups clean up after themselves
  proto_file.close();
  return true;
}

}  // namespace io
}  // namespace cblox

#endif  // CBLOX_IO_SUBMAP_IO_INL_H_
