#ifndef CBLOX_ROS_SUBMAP_CONVERSIONS_H
#define CBLOX_ROS_SUBMAP_CONVERSIONS_H

#include <cblox/core/common.h>
#include <voxblox_ros/conversions.h>
#include <geometry_msgs/Pose.h>
#include <minkindr_conversions/kindr_msg.h>

namespace cblox {

/*
inline void serializePoseUpdateToMsg(TsdfSubmap::ConstPtr submap_ptr,
                                 cblox_msgs::PoseUpdate* msg) {
  CHECK_NOTNULL(msg);
  CHECK_NOTNULL(submap_ptr);

  // Encode submap id in message
  msg->id = submap_ptr->getID();

  // Transform submap pose into message
  const Transformation submap_pose = submap_ptr->getPose();
  geometry_msgs::Pose pose_msg;
  tf::poseKindrToMsg(submap_pose.cast<double>(), &pose_msg);
  msg->pose = pose_msg;
}

inline bool deserializeMsgToPoseUpdate(cblox_msgs::PoseUpdate::Ptr msg_ptr,
    SubmapCollection<TsdfSubmap>::Ptr submap_collection_ptr) {
  CHECK_NOTNULL(submap_collection_ptr);
  // TODO: more checks to parse

  // read information in msg
  SubmapID submap_id = msg_ptr->id;
  kindr::minimal::QuatTransformationTemplate<double> pose;
  //Transformation submap_pose;
  tf::poseMsgToKindr(msg_ptr->pose, &pose);
  Transformation submap_pose = pose.cast<float>();

  // update pose of submap
  if (submap_collection_ptr->exists(submap_id)) {
    submap_collection_ptr->getSubMapPtrById(submap_id)->setPose(submap_pose);
  } else {
    ROS_WARN_STREAM("Pose update failed! (Submap " << submap_id <<
        " does not exist in collection!)");
    return false;
  }

  return true;
}
*/

inline void serializeSubmapToMsg(TsdfSubmap::ConstPtr submap_ptr,
    cblox_msgs::Submap* msg) {
  CHECK_NOTNULL(msg);
  CHECK_NOTNULL(submap_ptr);
//  ROS_INFO("Writing Submap into Message");

  // Encode submap id in message
  msg->id = submap_ptr->getID();

  // Transform submap pose into message
  const Transformation submap_pose = submap_ptr->getPose();
  geometry_msgs::Pose pose_msg;
  tf::poseKindrToMsg(submap_pose.cast<double>(), &pose_msg);
  msg->pose = pose_msg;

  // Transforms TSDF layer into message
  const bool only_updated = false;
  voxblox_msgs::Layer layer_msg;
  voxblox::serializeLayerAsMsg<TsdfVoxel>(
      submap_ptr->getTsdfMap().getTsdfLayer(), only_updated, &layer_msg);
  msg->layer = layer_msg;
}

inline bool deserializeMsgToSubmap(cblox_msgs::Submap::Ptr msg_ptr,
    SubmapCollection<TsdfSubmap>::Ptr submap_collection_ptr) {
  CHECK_NOTNULL(submap_collection_ptr);
  // TODO: more checks to parse
//  ROS_INFO("Parsing Message into Submap");

  // read information in msg
  SubmapID submap_id = msg_ptr->id;
  kindr::minimal::QuatTransformationTemplate<double> pose;
  //Transformation submap_pose;
  tf::poseMsgToKindr(msg_ptr->pose, &pose);
  Transformation submap_pose = pose.cast<float>();

  // create or change submap in collection
  if (submap_collection_ptr->exists(submap_id)) {
    submap_collection_ptr->getSubMapPtrById(submap_id)->setPose(submap_pose);

    // update submap
    voxblox::deserializeMsgToLayer(msg_ptr->layer,
        submap_collection_ptr->getSubMapPtrById(submap_id)->
        getTsdfMapPtr()->getTsdfLayerPtr());
  } else {
    // create new submap
    submap_collection_ptr->createNewSubMap(submap_pose, submap_id);
    // set TSDF map in submap collection
    voxblox::deserializeMsgToLayer(msg_ptr->layer,
        submap_collection_ptr->getSubMapPtrById(submap_id)->
        getTsdfMapPtr()->getTsdfLayerPtr());
  }
  return true;
}

}
#endif //CBLOX_ROS_SUBMAP_CONVERSIONS_H
