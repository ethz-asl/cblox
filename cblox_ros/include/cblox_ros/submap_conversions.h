#ifndef CBLOX_ROS_SUBMAP_CONVERSIONS_H
#define CBLOX_ROS_SUBMAP_CONVERSIONS_H

#include <cblox/core/common.h>
#include <voxblox_ros/conversions.h>
#include <geometry_msgs/Pose.h>
#include <minkindr_conversions/kindr_msg.h>

#include <cblox/core/submap_collection.h>

#include <cblox_msgs/MapHeader.h>
#include <cblox_msgs/MapPoseEstimate.h>
#include <cblox_msgs/MapLayer.h>

namespace cblox {

template <typename SubmapType>
std_msgs::Header generateHeaderMsg(
    const typename SubmapType::Ptr& submap_ptr, const ros::Time &timestamp);

template <typename SubmapType>
cblox_msgs::MapHeader generateSubmapHeaderMsg(
    const typename SubmapType::Ptr& submap_ptr);

template <typename SubmapType>
void serializeSubmapToMsg(typename SubmapType::Ptr submap_ptr,
    cblox_msgs::MapLayer* msg);

template <typename SubmapType>
bool deserializeMsgToSubmap(cblox_msgs::MapLayer* msg_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr);

}

#include "cblox_ros/submap_conversions_inl.h"

#endif //CBLOX_ROS_SUBMAP_CONVERSIONS_H
