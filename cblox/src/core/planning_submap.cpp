#include "cblox/core/planning_submap.h"

namespace cblox {

void PlanningSubmap::computeMapBounds() {
  Eigen::Vector3d lower_tmp, upper_tmp;
  voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                            &lower_bound_, &upper_bound_);
}

void PlanningSubmap::getGlobalMapBounds(Eigen::Vector3d* lower_bound,
                        Eigen::Vector3d* upper_bound) {
    // get corners of bounding cube
    std::vector<Eigen::Vector3d> bounds;
    bounds.emplace_back(lower_bound_);
    bounds.emplace_back(upper_bound_);
    bounds.emplace_back(Eigen::Vector3d(
        lower_bound_.x(), lower_bound_.y(), upper_bound_.z()));
    bounds.emplace_back(Eigen::Vector3d(
        lower_bound_.x(), upper_bound_.y(), lower_bound_.z()));
    bounds.emplace_back(Eigen::Vector3d(
        upper_bound_.x(), lower_bound_.y(), lower_bound_.z()));
    bounds.emplace_back(Eigen::Vector3d(
        lower_bound_.x(), upper_bound_.y(), upper_bound_.z()));
    bounds.emplace_back(Eigen::Vector3d(
        upper_bound_.x(), lower_bound_.y(), upper_bound_.z()));
    bounds.emplace_back(Eigen::Vector3d(
        upper_bound_.x(), upper_bound_.y(), lower_bound_.z()));

    // transform and save global extrema
    bool first = true;
    for (int i = 0; i < bounds.size(); i++) {
      bounds[i] = T_M_S_.cast<double>() * bounds[i];
      if (first) {
        first = false;
        *lower_bound = bounds[i];
        *upper_bound = bounds[i];
      } else {
        *lower_bound = bounds[i].cwiseMin(*lower_bound);
        *upper_bound = bounds[i].cwiseMax(*upper_bound);
      }
    }
  }

void PlanningSubmap::generateGlobalSparseGraph() {
  sparse_graph_.clear();
  sparse_graph_ = local_graph_;
  sparse_graph_.transformFrame(T_M_S_);
//  setupGraphPlanner();
  /*
  std::vector<int64_t> ids;
  local_graph_.getAllVertexIds(&ids);
  int num_nodes = ids.size();
  ids.clear();
  local_graph_.getAllEdgeIds(&ids);
  int num_edges = ids.size();
  ROS_INFO("local graph: %d nodes, %d edges",
           num_nodes, num_edges);
  ids.clear();
  sparse_graph_.getAllVertexIds(&ids);
  num_nodes = ids.size();
  ids.clear();
  sparse_graph_.getAllEdgeIds(&ids);
  num_edges = ids.size();
  ROS_INFO("global graph: %d nodes, %d edges",
           num_nodes, num_edges);
  */
}

void PlanningSubmap::setupGraphPlanner() {
  graph_planner_.setGraph(&sparse_graph_);
  graph_planner_.setup();
}

void PlanningSubmap::setupPrmGenerator(double robot_radius) {
  prm_generator_->setVerbose(false);
  prm_generator_->setMapDistanceCallback(
      std::bind(&PlanningSubmap::getMapDistance, this, std::placeholders::_1));
  prm_generator_->setCheckCollisionCallback(
      std::bind(&PlanningSubmap::checkCollision, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3));
  prm_generator_->setBounds(lower_bound_, upper_bound_);
  prm_generator_->setRobotRadius(robot_radius);
}

double PlanningSubmap::getMapDistance(const Eigen::Vector3d& position) {
  double distance;
  if (esdf_map_->getDistanceAtPosition(position, &distance)) {
    return distance;
  }
  return 0.0;
}
bool PlanningSubmap::checkCollision(const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal, const double& robot_radius) {
  Eigen::Vector3d position = start;
  Eigen::Vector3d direction = (goal - start).normalized();
  double distance = (goal - start).norm();
  double current_distance = 0;
  while (current_distance < distance) {
    double obstacle = getMapDistance(position);
    if (obstacle < robot_radius) {
      return true;
    }
    double step_size = std::max(1e-3, std::min(4.0, obstacle - robot_radius));
    position += step_size * direction;
    current_distance += step_size;
  }
  if (getMapDistance(goal) < robot_radius) {
    return true;
  }
  return false;
}

} // namespace cblox