#ifndef CBLOX_PLANNING_SUBMAP_H
#define CBLOX_PLANNING_SUBMAP_H

#include <cblox/core/tsdf_esdf_submap.h>
#include <voxblox/utils/planning_utils.h>
#include <voxblox_skeleton/skeleton.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <cblox/core/prm_generator.h>
#include <voxblox_skeleton/sparse_graph_planner.h>

namespace cblox {

class PlanningSubmap : public TsdfEsdfSubmap {
public:
  typedef std::shared_ptr<PlanningSubmap> Ptr;
  typedef std::shared_ptr<const PlanningSubmap> ConstPtr;

  explicit PlanningSubmap(Config config)
      : TsdfEsdfSubmap(config),
        skeleton_generator_(
            new voxblox::SkeletonGenerator(esdf_map_->getEsdfLayerPtr())),
        prm_generator_(new test_namespace::PrmGenerator()) { }

  explicit PlanningSubmap(
      const Transformation& T_M_S, SubmapID submap_id, Config config)
      : TsdfEsdfSubmap(T_M_S, submap_id, config),
        skeleton_generator_(
            new voxblox::SkeletonGenerator(esdf_map_->getEsdfLayerPtr())),
        prm_generator_(new test_namespace::PrmGenerator()) { }

  // map functions
  void computeMapBounds();
  void getMapBounds(Eigen::Vector3d* lower_bound, Eigen::Vector3d* upper_bound) {
    *lower_bound = lower_bound_;
    *upper_bound = upper_bound_;
  }
  void getGlobalMapBounds(Eigen::Vector3d* lower_bound,
                          Eigen::Vector3d* upper_bound);

  // skeleton functions
  void setLocalGraph(const voxblox::SparseGraph& graph) {
    local_graph_ = graph;
  };
  void generateGlobalSparseGraph();

  void setupPrmGenerator(double robot_radius);
  void clearSkeletonGenerator() {
    skeleton_generator_ =
        new voxblox::SkeletonGenerator(esdf_map_->getEsdfLayerPtr());
  }

  void setupGraphPlanner();

  // get functions
  voxblox::SkeletonGenerator* getSkeletonGenerator() {
    return skeleton_generator_;
  }
  test_namespace::PrmGenerator* getPrmGenerator() {
    return prm_generator_;
  }
  const voxblox::SparseGraph& getConstGlobalSparseGraph() {
    return sparse_graph_;
  };
  voxblox::SparseGraph& getGlobalSparseGraph() {
    return sparse_graph_;
  }
  voxblox::SparseGraphPlanner* getGraphPlanner() {
    return &graph_planner_;
  }

 private:
  double getMapDistance(const Eigen::Vector3d& position);
  bool checkCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
      const double& robot_radius);

  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;

  voxblox::SkeletonGenerator* skeleton_generator_;
  test_namespace::PrmGenerator* prm_generator_;

  voxblox::SparseGraph local_graph_;
  voxblox::SparseGraph sparse_graph_;
  voxblox::SparseGraphPlanner graph_planner_;
};

} // namespace cblox
#endif //CBLOX_PLANNING_SUBMAP_H