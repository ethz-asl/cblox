#include "cblox_ros/submap_conversions.h"
#include "cblox_ros/submap_conversions_inl.h"
#include <voxblox_ros/conversions.h>

namespace cblox {

template <>
void serializeSubmapToMsg<TsdfEsdfSubmap>(TsdfEsdfSubmap::Ptr submap_ptr,
                                          cblox_msgs::MapLayer* msg) {
  serializeSubmapToMsg<TsdfSubmap>(submap_ptr, msg);

  // set type to ESDF
  msg->type = 1;
  voxblox::serializeLayerAsMsg<EsdfVoxel>(
      submap_ptr->getEsdfMapPtr()->getEsdfLayer(), false, &msg->esdf_layer);
  msg->esdf_layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
}

SubmapID deserializeMsgToSubmapID(cblox_msgs::MapLayer* msg_ptr) {
  // read id
  return msg_ptr->map_header.id;
}
Transformation deserializeMsgToSubmapPose(cblox_msgs::MapLayer* msg_ptr) {
  // read pose
  kindr::minimal::QuatTransformationTemplate<double> pose;
  tf::poseMsgToKindr(msg_ptr->map_header.pose_estimate.map_pose, &pose);
  return pose.cast<float>();
}

template <>
bool deserializeMsgToSubmapContent<TsdfEsdfSubmap>(
    cblox_msgs::MapLayer* msg_ptr, TsdfEsdfSubmap::Ptr submap_ptr) {
  bool tsdf_success = deserializeMsgToSubmapContent<TsdfSubmap>(
      msg_ptr, static_cast<TsdfSubmap::Ptr>(submap_ptr));
  if (!tsdf_success) {
    return false;
  }

  bool esdf_success = voxblox::deserializeMsgToLayer(
      msg_ptr->esdf_layer, submap_ptr->getEsdfMapPtr()->getEsdfLayerPtr());
  // generate ESDF layer if necessary
  if (tsdf_success and !esdf_success) {
    submap_ptr->generateEsdf();
  }
  return true;
}

}