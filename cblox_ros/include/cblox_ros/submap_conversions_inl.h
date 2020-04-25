#ifndef CBLOX_ROS_SUBMAP_CONVERSIONS_INL_H
#define CBLOX_ROS_SUBMAP_CONVERSIONS_INL_H

#include <voxblox_ros/conversions.h>

#include <cblox_msgs/MapHeader.h>
#include <cblox_msgs/MapPoseEstimate.h>

namespace cblox {

template <typename SubmapType>
std_msgs::Header generateHeaderMsg(const typename SubmapType::Ptr& submap_ptr,
                                   const ros::Time& timestamp) {
  std_msgs::Header msg_header;
  msg_header.frame_id = "submap_" + std::to_string(submap_ptr->getID());
  msg_header.stamp = timestamp;
  return msg_header;
}

template <typename SubmapType>
cblox_msgs::MapHeader generateSubmapHeaderMsg(
    const typename SubmapType::Ptr& submap_ptr) {
  // Set the submap ID and type.
  cblox_msgs::MapHeader submap_header;
  submap_header.id = submap_ptr->getID();
  submap_header.is_submap = true;

  // Set the submap's start and end time.
  submap_header.start_time = ros::Time(submap_ptr->getMappingInterval().first);
  submap_header.end_time = ros::Time(submap_ptr->getMappingInterval().second);

  // Set the pose estimate and indicate what frame it's in.
  submap_header.pose_estimate.frame_id =
      "submap_" + std::to_string(submap_ptr->getID());
  tf::poseKindrToMsg(submap_ptr->getPose().template cast<double>(),
                     &submap_header.pose_estimate.map_pose);

  return submap_header;
}

template <typename SubmapType>
void serializePoseToMsg(typename SubmapType::Ptr submap_ptr,
                        cblox_msgs::MapPoseUpdate* msg) {
  CHECK_NOTNULL(msg);
  CHECK_NOTNULL(submap_ptr);

  ros::Time timestamp = ros::Time::now();
  // Fill in headers.
  msg->header = generateHeaderMsg<SubmapType>(submap_ptr, timestamp);
  // NOTE: Assuming that the submap IDs are equal to their index in storage.
  msg->map_headers[submap_ptr->getID()] =
      generateSubmapHeaderMsg<SubmapType>(submap_ptr);
}

// Note: Assumes that SubmapType contains a tsdf map.
template <typename SubmapType>
void serializeSubmapToMsg(typename SubmapType::Ptr submap_ptr,
                          cblox_msgs::MapLayer* msg) {
  CHECK_NOTNULL(msg);
  CHECK_NOTNULL(submap_ptr);

  ros::Time timestamp = ros::Time::now();
  // Fill in headers.
  msg->header = generateHeaderMsg<SubmapType>(submap_ptr, timestamp);
  msg->map_header = generateSubmapHeaderMsg<SubmapType>(submap_ptr);

  // Set type to TSDF.
  msg->type = static_cast<uint8_t>(MapLayerTypes::kTsdf);

  // Fill in TSDF layer.
  voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
      submap_ptr->getTsdfMapPtr()->getTsdfLayer(), false, &msg->tsdf_layer);
  msg->tsdf_layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
}

template <typename SubmapType>
typename SubmapType::Ptr deserializeMsgToSubmapPtr(
    cblox_msgs::MapLayer* msg_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr) {
  SubmapID submap_id = deserializeMsgToSubmapID(msg_ptr);
  if (!submap_collection_ptr->exists(submap_id)) {
    // Create new submap.
    submap_collection_ptr->createNewSubmap(Transformation(), submap_id);
  }
  return submap_collection_ptr->getSubmapPtr(submap_id);
}

// Note: Assumes that SubmapType contains a tsdf map.
template <typename SubmapType>
bool deserializeMsgToSubmapContent(cblox_msgs::MapLayer* msg_ptr,
                                   typename SubmapType::Ptr submap_ptr) {
  Transformation submap_pose = deserializeMsgToSubmapPose(msg_ptr);
  submap_ptr->setPose(submap_pose);

  // Read mapping interval.
  submap_ptr->startMappingTime(msg_ptr->map_header.start_time.toSec());
  submap_ptr->stopMappingTime(msg_ptr->map_header.end_time.toSec());

  return voxblox::deserializeMsgToLayer(
      msg_ptr->tsdf_layer, submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr());
}

template <typename SubmapType>
SubmapID deserializeMsgToSubmap(
    cblox_msgs::MapLayer* msg_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr) {
  CHECK_NOTNULL(submap_collection_ptr);
  if (!msg_ptr->map_header.is_submap) {
    return -1;
  }

  typename SubmapType::Ptr submap_ptr =
      deserializeMsgToSubmapPtr<SubmapType>(msg_ptr, submap_collection_ptr);
  deserializeMsgToSubmapContent<SubmapType>(msg_ptr, submap_ptr);

  return submap_ptr->getID();
}

}  // namespace cblox
#endif  // CBLOX_ROS_SUBMAP_CONVERSIONS_INL_H
