#ifndef CBLOX_IO_TRANSFORMATION_IO_H_
#define CBLOX_IO_TRANSFORMATION_IO_H_

#include <string>

#include "cblox/core/common.h"

namespace cblox {
namespace io {

bool SaveTransformationArray(
    const AlignedVector<Transformation>& transformation_array,
    const std::string& file_path);

bool LoadTransformationArray(
    const std::string& file_path,
    AlignedVector<Transformation>* transformation_array_ptr);

}  // namespace io
}  // namespace cblox

#endif  // CBLOX_IO_TRANSFORMATION_IO_H_
