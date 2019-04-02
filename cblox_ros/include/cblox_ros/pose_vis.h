#ifndef CBLOX_ROS_POSE_VIS_H_
#define CBLOX_ROS_POSE_VIS_H_

#include <geometry_msgs/PoseArray.h>

#include <cblox/core/common.h>

namespace cblox {

inline void posesToMsg(const TransformationVector& submap_poses,
                       geometry_msgs::PoseArray* pose_array_msg_ptr) {
  CHECK_NOTNULL(pose_array_msg_ptr);
  // Converting
  for (const Transformation& submap_pose : submap_poses) {
    geometry_msgs::Pose pose_msg;
    tf::poseKindrToMsg(submap_pose.cast<double>(), &pose_msg);
    pose_array_msg_ptr->poses.push_back(pose_msg);
  }
}

}  // namespace cblox

#endif  // CBLOX_ROS_POSE_VIS_H_
