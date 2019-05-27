#ifndef CBLOX_PLANNING_SUBMAP_H
#define CBLOX_PLANNING_SUBMAP_H

#include <voxblox/utils/planning_utils.h>
#include <voxblox_skeleton/skeleton.h>
#include <voxblox_skeleton/skeleton_generator.h>

namespace cblox {

class PlanningSubmap : public TsdfEsdfSubmap {
public:
  typedef std::shared_ptr<PlanningSubmap> Ptr;
  typedef std::shared_ptr<const PlanningSubmap> ConstPtr;

  PlanningSubmap(Config config)
      : TsdfEsdfSubmap(config),
        skeleton_generator_(esdf_map_->getEsdfLayerPtr()) { }

  PlanningSubmap(const Transformation& T_M_S, SubmapID submap_id, Config config)
      : TsdfEsdfSubmap(T_M_S, submap_id, config),
        skeleton_generator_(esdf_map_->getEsdfLayerPtr()) { }

  // map functions
  void computeMapBounds() {
    Eigen::Vector3d lower_tmp, upper_tmp;
    voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
        &lower_tmp, &upper_tmp);

    // get corners of bounding cube
    std::vector<Eigen::Vector3d> bounds;
    bounds.emplace_back(lower_tmp);
    bounds.emplace_back(upper_tmp);
    bounds.emplace_back(Eigen::Vector3d(
        lower_tmp.x(), lower_tmp.y(), upper_tmp.z()));
    bounds.emplace_back(Eigen::Vector3d(
        lower_tmp.x(), upper_tmp.y(), lower_tmp.z()));
    bounds.emplace_back(Eigen::Vector3d(
        upper_tmp.x(), lower_tmp.y(), lower_tmp.z()));
    bounds.emplace_back(Eigen::Vector3d(
        lower_tmp.x(), upper_tmp.y(), upper_tmp.z()));
    bounds.emplace_back(Eigen::Vector3d(
        upper_tmp.x(), lower_tmp.y(), upper_tmp.z()));
    bounds.emplace_back(Eigen::Vector3d(
        upper_tmp.x(), upper_tmp.y(), lower_tmp.z()));

    // transform and save global extrema
    bool first = true;
    for (int i = 0; i < bounds.size(); i++) {
      bounds[i] = T_M_S_.cast<double>() * bounds[i];
      if (first) {
        first = false;
        lower_bound_ = bounds[i];
        upper_bound_ = bounds[i];
      } else {
        lower_bound_ = bounds[i].cwiseMin(lower_bound_);
        upper_bound_ = bounds[i].cwiseMax(upper_bound_);
      }
    }
  }
  void getMapBounds(Eigen::Vector3d* lower_bound, Eigen::Vector3d* upper_bound) {
    *lower_bound = lower_bound_;
    *upper_bound = upper_bound_;
  }

  // skeleton functions
  void generateSkeleton() {
    skeleton_generator_.generateSkeleton();
  }
  void generateSparseGraph() {
    skeleton_generator_.generateSparseGraph();
  }

  // get functions
  voxblox::SkeletonGenerator* getSkeletonGenerator() {
    return &skeleton_generator_;
  }
  /*
  voxblox::SparseSkeletonGraph getGlobalSparseGraph() {
    voxblox::SparseSkeletonGraph local_graph =
        skeleton_generator_.getSparseGraph();
    voxblox::SparseSkeletonGraph global_graph = local_graph;
    ROS_INFO_STREAM("before transform: [" << local_graph.getVertex(0).point.x()
        << ", " << local_graph.getVertex(0).point.y()
        << ", " << local_graph.getVertex(0).point.z() << "]");
    global_graph.transformFrame(T_M_S_);
    ROS_INFO_STREAM("after transform:  " << local_graph.getVertex(0).point.x()
        << ", " << local_graph.getVertex(0).point.y()
        << ", " << local_graph.getVertex(0).point.z() << "]");
    ROS_INFO_STREAM("after transform:  " << global_graph.getVertex(0).point.x()
        << ", " << global_graph.getVertex(0).point.y()
        << ", " << global_graph.getVertex(0).point.z() << "]");
    ROS_INFO_STREAM(T_M_S_);
    return global_graph;
  }
  */
  /*
  voxblox::Skeleton getSkeletonLayerPtr() {
    return skeleton_generator_.getSkeleton();
  }
  voxblox::SparseSkeletonGraph getSparseGraph() {
    return skeleton_generator_.getSparseGraph();
  }
  */

 private:
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;

  voxblox::SkeletonGenerator skeleton_generator_;
};

} // namespace cblox
#endif //CBLOX_PLANNING_SUBMAP_H