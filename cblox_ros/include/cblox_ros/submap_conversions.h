#ifndef CBLOX_ROS_SUBMAP_CONVERSIONS_H
#define CBLOX_ROS_SUBMAP_CONVERSIONS_H

#include <geometry_msgs/Pose.h>
#include <minkindr_conversions/kindr_msg.h>

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>

#include <cblox_msgs/MapLayer.h>
#include <cblox_msgs/MapPoseUpdates.h>

namespace cblox {

enum class MapLayerTypes : uint8_t { kTsdf = 0u, kEsdf = 1u };

template <typename SubmapType>
std_msgs::Header generateHeaderMsg(const SubmapType& submap,
                                   const ros::Time& timestamp);

template <typename SubmapType>
cblox_msgs::MapHeader generateSubmapHeaderMsg(const SubmapType& submap);

template <typename SubmapType>
void serializePoseToMsg(const SubmapType& submap, cblox_msgs::MapHeader* msg);

template <typename SubmapType>
void serializeSubmapToMsg(const SubmapType& submap, cblox_msgs::MapLayer* msg);
template <>
void serializeSubmapToMsg<TsdfEsdfSubmap>(const TsdfEsdfSubmap& submap,
                                          cblox_msgs::MapLayer* msg);

template <typename SubmapType>
SubmapID deserializeMsgToSubmap(
    cblox_msgs::MapLayer* msg_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr);

template <typename SubmapType>
typename SubmapType::Ptr deserializeMsgToSubmapPtr(
    cblox_msgs::MapLayer* msg_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr);

template <typename SubmapType>
bool deserializeMsgToSubmapContent(cblox_msgs::MapLayer* msg_ptr,
                                   typename SubmapType::Ptr submap_ptr);
template <>
bool deserializeMsgToSubmapContent<TsdfEsdfSubmap>(
    cblox_msgs::MapLayer* msg_ptr, TsdfEsdfSubmap::Ptr submap_ptr);

void deserializeMsgToPose(const cblox_msgs::MapPoseUpdates& msg,
                          SubmapIdPoseMap* id_pose_map);

SubmapID deserializeMsgToSubmapID(cblox_msgs::MapLayer* msg_ptr);

Transformation deserializeMsgToSubmapPose(cblox_msgs::MapLayer* msg_ptr);

}  // namespace cblox

#endif  // CBLOX_ROS_SUBMAP_CONVERSIONS_H

#include "cblox_ros/submap_conversions_inl.h"
