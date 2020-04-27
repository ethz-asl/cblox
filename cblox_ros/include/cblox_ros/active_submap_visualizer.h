#ifndef CBLOX_ROS_ACTIVE_SUBMAP_VISUALIZER_H_
#define CBLOX_ROS_ACTIVE_SUBMAP_VISUALIZER_H_

#include <memory>

#include <visualization_msgs/Marker.h>

#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox_ros/mesh_vis.h>

#include <cblox/core/submap_collection.h>
#include <cblox/mesh/submap_mesher.h>

namespace cblox {

using voxblox::Color;
using voxblox::MeshIntegrator;
using voxblox::MeshIntegratorConfig;
using voxblox::MeshLayer;

class ActiveSubmapVisualizer {
 public:
  typedef std::shared_ptr<ActiveSubmapVisualizer> Ptr;
  typedef std::shared_ptr<const ActiveSubmapVisualizer> ConstPtr;

  // Constructor
  ActiveSubmapVisualizer(const MeshIntegratorConfig& mesh_config,
                         const std::shared_ptr<SubmapCollectionInterface>&
                             tsdf_submap_collection_ptr)
      : mesh_config_(mesh_config),
        tsdf_submap_collection_ptr_(tsdf_submap_collection_ptr),
        color_cycle_length_(kDefaultColorCycleLength),
        current_color_idx_(0),
        verbose_(false),
        opacity_(1.0) {}

  void switchToSubmap(const SubmapID submap_id);
  void switchToActiveSubmap();

  void updateMeshLayer();

  void getDisplayMesh(visualization_msgs::Marker* marker_ptr);
  MeshLayer::Ptr getDisplayMeshLayer();

  void setVerbose(const bool& verbose) { verbose_ = verbose; }
  void setOpacity(const float& opacity) { opacity_ = opacity; }

 private:
  // Functions called when swapping active submaps
  void createMeshLayer();
  void recoverMeshLayer();
  void updateIntegrator();

  // The active mesh is produced in the submap frame (S), and is transformed
  // into the global frame (G).
  void transformMeshLayerToGlobalFrame(const MeshLayer& mesh_layer_S,
                                       MeshLayer* mesh_layer_G_ptr) const;
  void colorMeshWithCurrentIndex(MeshLayer* mesh_layer_ptr) const;

  // Config
  const MeshIntegratorConfig mesh_config_;

  // The mesh layer for the active submap
  std::shared_ptr<MeshLayer> active_submap_mesh_layer_ptr_;
  int active_submap_color_idx_;

  // The integrator
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> active_submap_mesh_integrator_ptr_;

  // The submap collection
  std::shared_ptr<SubmapCollectionInterface> tsdf_submap_collection_ptr_;
  SubmapID active_submap_id_;

  // Storing the mesh layers
  std::map<SubmapID, std::shared_ptr<MeshLayer>> mesh_layers_;
  std::map<SubmapID, int> mesh_color_indices_;

  // Color stuff
  const int color_cycle_length_;
  int current_color_idx_;

  bool verbose_;
  float opacity_;
};

}  // namespace cblox

#endif  // CBLOX_ROS_ACTIVE_SUBMAP_VISUALIZER_H_
