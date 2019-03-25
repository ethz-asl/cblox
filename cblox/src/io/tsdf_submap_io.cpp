#include <string>
#include "cblox/core/tsdf_submap.h"
#include "cblox/io/tsdf_submap_io.h"

namespace cblox {
namespace io {

bool SaveTsdfSubmapCollection(
    const SubmapCollection<TsdfSubmap> &tsdf_submap_collection,
    const std::string &file_path) {
  return tsdf_submap_collection.saveToFile(file_path);
}

}  // namespace io
}  // namespace cblox
