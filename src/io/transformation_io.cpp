#include <iostream>
#include <string>

#include <voxblox/utils/protobuf_utils.h>

#include "./QuatTransformation.pb.h"

#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace io {

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
  if (!voxblox::utils::writeProtoMsgCountToStream(num_messages, &proto_file)) {
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
    if (!voxblox::utils::writeProtoMsgToStream(transformation_proto,
                                               &proto_file)) {
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
  if (!voxblox::utils::readProtoMsgCountToStream(&proto_file, &num_protos,
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
    if (!voxblox::utils::readProtoMsgFromStream(
            &proto_file, &transformation_proto, &tmp_byte_offset)) {
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
}  // namespace cblox
