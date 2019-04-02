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
  // T_G_C - Transformation between Camera (C) and Global tracking frame (G).
  void addPose(const Transformation& T_G_C) { T_G_C_array_.push_back(T_G_C); }

  // Gets the trajectory as a message for visualization
  void getTrajectoryMsg(nav_msgs::Path* path_msg_ptr) const;

 private:
  // The vector of trajectory poses
  // Note(alexmillane): In the future these poses will have to be updated to
  // reflect changes in the SLAM graph.
  TransformationVector T_G_C_array_;
};

}  // namespace cblox

#endif  // CBLOX_ROS_TRAJECTORY_VISUALIZER_H_
