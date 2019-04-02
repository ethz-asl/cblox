#ifndef CBLOX_ROS_ACTIVE_SUBMAP_MESHER_H_
#define CBLOX_ROS_ACTIVE_SUBMAP_MESHER_H_

#include <memory>
/*#include <vector>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>

#include "cblox/core/common.h"
#include "cblox/core/tsdf_submap.h"
*/

#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>

#include "cblox/core/submap_collection.h"

namespace cblox {

// using voxblox::Mesh;
using voxblox::MeshLayer;
using voxblox::MeshIntegrator;
using voxblox::MeshIntegratorConfig;
using voxblox::Color;
// using voxblox::VertexIndex;

constexpr int kDefaultColorCycleLength = 20;

class ActiveSubmapMesher {
 public:
  typedef std::shared_ptr<ActiveSubmapMesher> Ptr;
  typedef std::shared_ptr<const ActiveSubmapMesher> ConstPtr;

  // Constructor
  ActiveSubmapMesher(const MeshIntegratorConfig& mesh_config,
                     const std::shared_ptr<SubmapCollection<TsdfSubmap>>&
                         tsdf_submap_collection_ptr)
      : mesh_config_(mesh_config),
        tsdf_submap_collection_ptr_(tsdf_submap_collection_ptr),
        color_cycle_length_(kDefaultColorCycleLength),
        current_color_idx_(-1) {}

  void activateLatestSubmap();

  void updateMeshLayer();

  // The active mesh is produced in the submap frame (S), and is transformed
  // into the global frame (G).
  void transformMeshLayerToGlobalFrame(const MeshLayer& mesh_layer_S,
                                       MeshLayer* mesh_layer_G_ptr) const;
  void colorMeshWithCurrentIndex(MeshLayer* mesh_layer_ptr) const;

  // Note(alexmillane): Return by const ptr for voxblox vis
  const std::shared_ptr<MeshLayer> getDisplayMesh();

 private:
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

#endif  // CBLOX_ROS_ACTIVE_SUBMAP_MESHER_H_
