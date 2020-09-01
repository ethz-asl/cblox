#include "cblox_ros/submap_conversions.h"

#include <voxblox_ros/conversions.h>

#include "cblox_ros/submap_conversions_inl.h"

namespace cblox {

template <>
void serializeSubmapToMsg<TsdfEsdfSubmap>(const TsdfEsdfSubmap& submap,
                                          cblox_msgs::MapLayer* msg) {
  serializeSubmapToMsg<TsdfSubmap>(submap, msg);

  // set type to ESDF
  msg->type = static_cast<uint8_t>(MapLayerTypes::kEsdf);
  voxblox::serializeLayerAsMsg<EsdfVoxel>(submap.getEsdfMap().getEsdfLayer(),
                                          false, &msg->esdf_layer);
  msg->esdf_layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
}

void deserializeMsgToPose(const cblox_msgs::MapPoseUpdates& msg,
                          SubmapIdPoseMap* id_pose_map) {
  CHECK_NOTNULL(id_pose_map);

  for (const cblox_msgs::MapHeader& pose_msg : msg.map_headers) {
    // Read id.
    SubmapID submap_id = pose_msg.id;

    // Read pose.
    kindr::minimal::QuatTransformationTemplate<double> pose;
    tf::poseMsgToKindr(pose_msg.pose_estimate.map_pose, &pose);
    Transformation submap_pose = pose.cast<float>();

    id_pose_map->emplace(submap_id, submap_pose);
  }
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

}  // namespace cblox
