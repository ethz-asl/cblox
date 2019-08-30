#ifndef CBLOX_ROS_SUBMAP_CONVERSIONS_H
#define CBLOX_ROS_SUBMAP_CONVERSIONS_H

#include <cblox/core/common.h>
#include <geometry_msgs/Pose.h>
#include <minkindr_conversions/kindr_msg.h>

#include <cblox/core/submap_collection.h>

#include <cblox_msgs/MapLayer.h>
#include <cblox_msgs/MapPoseUpdate.h>

namespace cblox {

template <typename SubmapType>
std_msgs::Header generateHeaderMsg(
    const typename SubmapType::Ptr& submap_ptr, const ros::Time &timestamp);

template <typename SubmapType>
cblox_msgs::MapHeader generateSubmapHeaderMsg(
    const typename SubmapType::Ptr& submap_ptr);

template<typename SubmapType>
void serializePoseToMsg(typename SubmapType::Ptr submap_ptr,
    cblox_msgs::MapHeader* msg);

template<typename SubmapType>
SubmapID deserializeMsgToPose(const cblox_msgs::MapPoseUpdate* msg,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr);

template <typename SubmapType>
void serializeSubmapToMsg(typename SubmapType::Ptr submap_ptr,
    cblox_msgs::MapLayer* msg);

template <typename SubmapType>
SubmapID deserializeMsgToSubmap(cblox_msgs::MapLayer* msg_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr);

}

#include "cblox_ros/submap_conversions_inl.h"

#endif //CBLOX_ROS_SUBMAP_CONVERSIONS_H
