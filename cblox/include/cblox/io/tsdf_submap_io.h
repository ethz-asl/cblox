#ifndef CBLOX_IO_TSDF_SUBMAP_IO_H_
#define CBLOX_IO_TSDF_SUBMAP_IO_H_

#include <string>

#include "cblox/core/common.h"
#include "cblox/core/submap_collection.h"

#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace io {

template <typename SubmapType>
bool SaveSubmapCollection(
    const SubmapCollection<SubmapType>& submap_collection,
    const std::string& file_path);

template <typename SubmapType>
bool LoadSubmapCollection(
    const std::string& file_path,
    typename SubmapCollection<SubmapType>::Ptr* tsdf_submap_collection_ptr);

// we assume most SubmapTypes to have an ESDF map
template<typename SubmapType>
bool LoadSubmapFromStream(
    std::fstream* proto_file_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr,
    uint32_t* tmp_byte_offset_ptr);


}  // namespace io
}  // namespace cblox

#include "cblox/io/tsdf_submap_io_inl.h"

#endif  // CBLOX_IO_TSDF_SUBMAP_IO_H_
