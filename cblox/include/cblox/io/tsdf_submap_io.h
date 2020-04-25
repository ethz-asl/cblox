#ifndef CBLOX_IO_TSDF_SUBMAP_IO_H_
#define CBLOX_IO_TSDF_SUBMAP_IO_H_

#include <string>

#include "cblox/core/common.h"
#include "cblox/core/submap_collection.h"

#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace io {

template <typename SubmapType>
bool LoadSubmapCollection(
    const std::string& file_path,
    typename SubmapCollection<SubmapType>::Ptr* tsdf_submap_collection_ptr);

  // we assume most SubmapTypes to have an ESDF map
template<typename SubmapType>
bool LoadSubmapFromStream(
    std::fstream* proto_file_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr,
    uint64_t* tmp_byte_offset_ptr);
template<>
bool LoadSubmapFromStream<cblox::TsdfSubmap>(
    std::fstream* proto_file_ptr,
    typename SubmapCollection<cblox::TsdfSubmap>::Ptr submap_collection_ptr,
    uint64_t* tmp_byte_offset_ptr);

template <typename SubmapType>
bool SaveSubmapCollection(
    const SubmapCollection<SubmapType>& submap_collection,
    const std::string& file_path);

}  // namespace io
}  // namespace cblox

#endif  // CBLOX_IO_TSDF_SUBMAP_IO_H_

#include "cblox/io/tsdf_submap_io_inl.h"
