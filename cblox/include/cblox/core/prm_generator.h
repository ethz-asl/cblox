#ifndef CBLOX_CBLOX_PRM_PLANNER_H
#define CBLOX_CBLOX_PRM_PLANNER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <voxblox_skeleton/sparse_graph.h>
//#include <voxblox_skeleton/sparse_graph_planner.h>

namespace test_namespace {

class PrmGenerator {
public:
  PrmGenerator();
  ~PrmGenerator() = default;

  void setBounds(const Eigen::Vector3d& lower_bound,
                 const Eigen::Vector3d& upper_bound) {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  };
  void setRobotRadius(double radius);
  void setMapDistanceCallback(
      const std::function<double(const Eigen::Vector3d& position)>& function);
  void setCheckCollisionCallback(const std::function<bool(
      const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
      const double& robot_radius)>& function);

  void growRoadmap(const ros::Duration& duration, const bool& incremental);

  const voxblox::SparseGraph& getRoadmap() const {return roadmap_;};
  void setRoadmap(const voxblox::SparseGraph& roadmap) {roadmap_ = roadmap;};

  void setVerbose(bool verbose) {verbose_ = verbose;};

protected:
  voxblox::SparseGraph roadmap_;
//  voxblox::SparseGraphPlanner planner_;

  ros::Time current_time_;
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;
  double robot_radius_;

  std::function<double(const Eigen::Vector3d& position)> map_distance_function_;
  std::function<bool(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
      const double robot_radius)> check_collision_function_;

  bool verbose_;
};

}

#endif //CBLOX_CBLOX_PRM_PLANNER_H
