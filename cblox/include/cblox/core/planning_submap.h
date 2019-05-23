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
    voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
        &lower_bound_, &upper_bound_);
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