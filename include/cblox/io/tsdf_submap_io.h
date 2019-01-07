#ifndef CBLOX_IO_TSDF_SUBMAP_IO_H_
#define CBLOX_IO_TSDF_SUBMAP_IO_H_

#include <string>

#include "cblox/core/common.h"
#include "cblox/core/submap_collection.h"

namespace cblox {
namespace io {

template <typename SubmapType>
bool SaveTsdfSubmapCollection(
    const SubmapCollection<SubmapType> &tsdf_submap_collection,
    const std::string &file_path);

template <typename SubmapType>
bool LoadTsdfSubmapFromStream(
    std::fstream *proto_file_ptr,
    typename SubmapCollection<SubmapType>::Ptr tsdf_submap_collection_ptr,
    uint32_t *tmp_byte_offset_ptr);

template <typename SubmapType>
bool LoadTsdfSubmapCollection(
    const std::string &file_path,
    typename SubmapCollection<SubmapType>::Ptr *tsdf_submap_collection_ptr);

}  // namespace io
}  // namespace cblox

#endif  // CBLOX_IO_TSDF_SUBMAP_IO_H_
