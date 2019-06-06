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
  global_skeleton_graph_ = skeleton_generator_.getSparseGraph();
  global_skeleton_graph_.transformFrame(T_M_S_);
  setupGraphPlanner();
}

void PlanningSubmap::setupGraphPlanner() {
  skeleton_graph_planner_.setGraph(&global_skeleton_graph_);
  skeleton_graph_planner_.setup();
}

} // namespace cblox