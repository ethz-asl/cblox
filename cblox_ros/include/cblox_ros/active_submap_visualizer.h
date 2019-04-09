#ifndef CBLOX_ROS_ACTIVE_SUBMAP_VISUALIZER_H_
#define CBLOX_ROS_ACTIVE_SUBMAP_VISUALIZER_H_

#include <memory>

#include <visualization_msgs/Marker.h>

#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox_ros/mesh_vis.h>

#include <cblox/core/submap_collection.h>

namespace cblox {

using voxblox::MeshLayer;
using voxblox::MeshIntegrator;
using voxblox::MeshIntegratorConfig;
using voxblox::Color;

constexpr int kDefaultColorCycleLength = 20;

class ActiveSubmapVisualizer {
 public:
  typedef std::shared_ptr<ActiveSubmapVisualizer> Ptr;
  typedef std::shared_ptr<const ActiveSubmapVisualizer> ConstPtr;

  // Constructor
  ActiveSubmapVisualizer(const MeshIntegratorConfig& mesh_config,
                         const std::shared_ptr<SubmapCollection<TsdfSubmap>>&
                             tsdf_submap_collection_ptr)
      : mesh_config_(mesh_config),
        tsdf_submap_collection_ptr_(tsdf_submap_collection_ptr),
        color_cycle_length_(kDefaultColorCycleLength),
        current_color_idx_(-1) {}

  void activateLatestSubmap();

  void updateMeshLayer();

  void getDisplayMesh(visualization_msgs::Marker* marker_ptr);
  MeshLayer::Ptr getDisplayMeshLayer();

private:
  // The active mesh is produced in the submap frame (S), and is transformed
  // into the global frame (G).
  void transformMeshLayerToGlobalFrame(const MeshLayer& mesh_layer_S,
                                       MeshLayer* mesh_layer_G_ptr) const;
  void colorMeshWithCurrentIndex(MeshLayer* mesh_layer_ptr) const;

  // Config
  const MeshIntegratorConfig mesh_config_;

  // The mesh layer for the active submap
  std::shared_ptr<MeshLayer> active_submap_mesh_layer_ptr_;

  // The integrator
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> active_submap_mesh_integrator_ptr_;

  // The submap collection
  std::shared_ptr<SubmapCollection<TsdfSubmap>> tsdf_submap_collection_ptr_;

  // Color stuff
  const int color_cycle_length_;
  int current_color_idx_;
};

}  // namespace cblox

#endif  // CBLOX_ROS_ACTIVE_SUBMAP_VISUALIZER_H_
