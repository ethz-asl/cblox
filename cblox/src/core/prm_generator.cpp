#include "cblox/core/prm_generator.h"

#include <mav_trajectory_generation/timing.h>
#include <mav_planning_common/utils.h>

namespace test_namespace {

  PrmGenerator::PrmGenerator() : robot_radius_(0.0), verbose_(false) {}

  void PrmGenerator::setRobotRadius(double radius) {
    robot_radius_ = radius;
  }

  void PrmGenerator::setMapDistanceCallback(
      const std::function<double(const Eigen::Vector3d &position)>& function) {
    map_distance_function_ = function;
  }

  void PrmGenerator::setCheckCollisionCallback(const std::function<bool(
      const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
      const double& robot_radius)>& function) {
    check_collision_function_ = function;
  }

  void PrmGenerator::growRoadmap(const ros::Duration& duration) {
    roadmap_.clear();
    double max_dist = 3.0;
    current_time_ = ros::Time::now();
    int num_nodes = 0;
    int num_edges = 0;
    while (ros::Time::now() - current_time_ < duration) {
//    while (num_nodes < 100) {
      // randomly sample
      mav_trajectory_generation::timing::Timer
          get_sample_timer("prm/grow_roadmap/get_sample");
      voxblox::Point new_state(mav_planning::randMToN(lower_bound_.x(), upper_bound_.x()),
                               mav_planning::randMToN(lower_bound_.y(), upper_bound_.y()),
                               mav_planning::randMToN(lower_bound_.z(), upper_bound_.z()));
      if (map_distance_function_(new_state.cast<double>()) < robot_radius_) {
        get_sample_timer.Stop();
        continue;
      }
      get_sample_timer.Stop();

      // find neighbors in graph so far
      mav_trajectory_generation::timing::Timer
          find_neighbors_timer("prm/grow_roadmap/find_neighbors");
      std::vector<uint64_t> neighbors;
      std::vector<long> vertex_ids;
      roadmap_.getAllVertexIds(&vertex_ids);
      for (long vertex_id : vertex_ids) {
        if ((new_state - roadmap_.getVertex(vertex_id).point).norm() < max_dist) {
          neighbors.emplace_back(vertex_id);
        }
      }
      find_neighbors_timer.Stop();

      // add vertex
      voxblox::GraphVertex new_vertex;
      new_vertex.distance = map_distance_function_(new_state.cast<double>());
      new_vertex.point = new_state;
      long new_vertex_id = roadmap_.addVertex(new_vertex);
      num_nodes++;
      // add edges to neighbors
      mav_trajectory_generation::timing::Timer
          connect_neighbors_timer("prm/grow_roadmap/connect_neighbors");
      for (int vertex_id : neighbors) {
        const voxblox::GraphVertex &neighbor = roadmap_.getVertex(vertex_id);
        if (!check_collision_function_(new_state.cast<double>(), neighbor.point.cast<double>(),
                robot_radius_)) {
          voxblox::GraphEdge new_edge;
          new_edge.start_distance = new_vertex.distance;
          new_edge.start_point = new_vertex.point;
          new_edge.start_vertex = new_vertex_id;
          new_edge.end_point = neighbor.point;
          new_edge.end_distance = neighbor.distance;
          new_edge.end_vertex = neighbor.vertex_id;
          roadmap_.addEdge(new_edge);
          num_edges++;
        }
      }
      connect_neighbors_timer.Stop();
    }
    bool verbose_ = true;
    if (verbose_) {
      ROS_INFO("[PrmGenerator] %d nodes and %d edges", num_nodes, num_edges);
    }
  }

}