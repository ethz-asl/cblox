#ifndef CBLOX_UTILS_QUAT_TRANSFORMATION_PROTOBUF_UTILS_H_
#define CBLOX_UTILS_QUAT_TRANSFORMATION_PROTOBUF_UTILS_H_

#include "cblox/QuatTransformation.pb.h"
#include "cblox/core/common.h"

namespace cblox {
namespace conversions {

void transformKindrToProto(const Transformation& transformation,
                           QuatTransformationProto* quat_transformation_proto);

void transformProtoToKindr(
    const QuatTransformationProto& quat_transformation_proto,
    Transformation* transformation);

}  // namespace conversions
}  // namespace cblox

#endif  // CBLOX_UTILS_QUAT_TRANSFORMATION_PROTOBUF_UTILS_H_
