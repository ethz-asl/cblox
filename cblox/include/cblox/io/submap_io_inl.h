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
  return SubmapCollection<SubmapType>::LoadFromFile(file_path,
                                                    submap_collection_ptr);
}

}  // namespace io
}  // namespace cblox

#endif  // CBLOX_IO_SUBMAP_IO_INL_H_
