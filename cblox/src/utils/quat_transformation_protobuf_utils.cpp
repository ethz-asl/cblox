#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {
namespace conversions {

void transformKindrToProto(const Transformation& transformation,
                           QuatTransformationProto* quat_transformation_proto) {
  CHECK_NOTNULL(quat_transformation_proto);
  Eigen::Vector3 t = transformation.getPosition();
  Quaternion q = transformation.getRotation();
  quat_transformation_proto->set_tx(t.x());
  quat_transformation_proto->set_ty(t.y());
  quat_transformation_proto->set_tz(t.z());
  quat_transformation_proto->set_qw(q.w());
  quat_transformation_proto->set_qx(q.x());
  quat_transformation_proto->set_qy(q.y());
  quat_transformation_proto->set_qz(q.z());
}

void transformProtoToKindr(
    const QuatTransformationProto& quat_transformation_proto,
    Transformation* transformation) {
  CHECK_NOTNULL(transformation);
  Eigen::Vector3 t(quat_transformation_proto.tx(),
                   quat_transformation_proto.ty(),
                   quat_transformation_proto.tz());
  Quaternion q(quat_transformation_proto.qw(), quat_transformation_proto.qx(),
               quat_transformation_proto.qy(), quat_transformation_proto.qz());
  *transformation = Transformation(q, t);
}

}  // namespace conversions
}  // namespace cblox
