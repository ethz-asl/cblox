#ifndef CBLOX_IO_TSDF_SUBMAP_IO_H_
#define CBLOX_IO_TSDF_SUBMAP_IO_H_

#include <string>

#include "cblox/core/common.h"
#include "cblox/core/submap_collection.h"

namespace cblox {
namespace io {

bool SaveTsdfSubmapCollection(
    const SubmapCollection<TsdfSubmap> &tsdf_submap_collection,
    const std::string &file_path);

bool LoadTsdfSubmapFromStream(
    std::fstream *proto_file_ptr,
    typename SubmapCollection<TsdfSubmap>::Ptr tsdf_submap_collection_ptr,
    uint32_t *tmp_byte_offset_ptr);

bool LoadTsdfSubmapCollection(
    const std::string &file_path,
    typename SubmapCollection<TsdfSubmap>::Ptr *tsdf_submap_collection_ptr);

}  // namespace io
}  // namespace cblox

#endif  // CBLOX_IO_TSDF_SUBMAP_IO_H_
