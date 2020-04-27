#ifndef CBLOX_IO_SUBMAP_IO_H_
#define CBLOX_IO_SUBMAP_IO_H_

#include <string>

#include "cblox/core/common.h"
#include "cblox/core/submap_collection.h"

#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace io {

template <typename SubmapType>
bool LoadSubmapCollection(
    const std::string& file_path,
    typename SubmapCollection<SubmapType>::Ptr* submap_collection_ptr) {
  return SubmapCollection<SubmapType>::LoadFromFile(file_path,
                                                    submap_collection_ptr);
}

template <typename SubmapType>
bool SaveSubmapCollection(const SubmapCollection<SubmapType>& submap_collection,
                          const std::string& file_path) {
  return submap_collection.saveToFile(file_path);
}

}  // namespace io
}  // namespace cblox

#endif  // CBLOX_IO_SUBMAP_IO_H_
