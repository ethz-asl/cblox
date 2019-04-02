#ifndef CBLOX_ROS_TRAJECTORY_VISUALIZER_H_
#define CBLOX_ROS_TRAJECTORY_VISUALIZER_H_

#include <nav_msgs/Path.h>

#include <cblox/core/common.h>

namespace cblox {

class TrajectoryVisualizer {
 public:
  typedef std::shared_ptr<TrajectoryVisualizer> Ptr;
  typedef std::shared_ptr<const TrajectoryVisualizer> ConstPtr;

  // Constructor
  TrajectoryVisualizer() {}

  // Adds a pose to the trajectory
  void addPose(const Transformation& T_W_C) { T_W_C_array_.push_back(T_W_C); }

  // Gets the trajectory as a message for visualization
  void getTrajectoryMsg(nav_msgs::Path* path_msg_ptr) const;

 private:
  // The vector of trajectory poses
  TransformationVector T_W_C_array_;
};

}  // namespace cblox

#endif  // CBLOX_ROS_TRAJECTORY_VISUALIZER_H_
