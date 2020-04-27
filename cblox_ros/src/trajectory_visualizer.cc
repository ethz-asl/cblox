#include "cblox_ros/trajectory_visualizer.h"

#include <geometry_msgs/PoseStamped.h>

#include "cblox_ros/pose_vis.h"

namespace cblox {

void TrajectoryVisualizer::getTrajectoryMsg(
    nav_msgs::Path* path_msg_ptr) const {
  CHECK_NOTNULL(path_msg_ptr);
  // Note(alexmillane): The trajectory is being regenerated at each published
  // pose. This could become heavy... Perhaps building up the message
  // incrementally might be a good idea.
  for (const Transformation& T_G_C : T_G_C_array_) {
    geometry_msgs::PoseStamped pose_stamped;
    tf::poseKindrToMsg(T_G_C.cast<double>(), &pose_stamped.pose);
    path_msg_ptr->poses.push_back(pose_stamped);
  }
}

}  // namespace cblox
