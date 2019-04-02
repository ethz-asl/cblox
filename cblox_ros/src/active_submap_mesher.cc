#include "cblox_ros/active_submap_mesher.h"

#include <cblox/mesh/submap_mesher.h>

namespace cblox {

void ActiveSubmapMesher::activateLatestSubmap() {
  // New mesh
  active_submap_mesh_layer_ptr_.reset(
      new voxblox::MeshLayer(tsdf_submap_collection_ptr_->block_size()));
  // New integrator operating on the mesh.
  active_submap_mesh_integrator_ptr_.reset(
      new voxblox::MeshIntegrator<TsdfVoxel>(
          mesh_config_,
          tsdf_submap_collection_ptr_->getActiveTsdfMapPtr()->getTsdfLayerPtr(),
          active_submap_mesh_layer_ptr_.get()));
  // New color for this map
  current_color_idx_ = (current_color_idx_ + 1) % color_cycle_length_;
}

void ActiveSubmapMesher::updateMeshLayer() {
  // Updating the mesh layer
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  active_submap_mesh_integrator_ptr_->generateMesh(only_mesh_updated_blocks,
                                                   clear_updated_flag);
}

void ActiveSubmapMesher::transformMeshLayerToGlobalFrame(
    const MeshLayer& mesh_layer_S, MeshLayer* mesh_layer_G_ptr) const {
  CHECK_NOTNULL(mesh_layer_G_ptr);
  // Transforming all triangles in the mesh and adding to the combined layer
  const Transformation& T_G_S =
      tsdf_submap_collection_ptr_->getActiveSubMapPose();
  SubmapMesher::transformAndAddTrianglesToLayer(mesh_layer_S, T_G_S,
                                                mesh_layer_G_ptr);
}

void ActiveSubmapMesher::colorMeshWithCurrentIndex(
    MeshLayer* mesh_layer_ptr) const {
  CHECK_NOTNULL(mesh_layer_ptr);
  // Coloring
  double color_map_idx = static_cast<double>(current_color_idx_) /
                         static_cast<double>(color_cycle_length_ - 1);
  const Color color = voxblox::rainbowColorMap(color_map_idx);
  SubmapMesher::colorMeshLayer(color, mesh_layer_ptr);
}

const std::shared_ptr<MeshLayer> ActiveSubmapMesher::getDisplayMesh() {
  // Transforming the mesh layer into G.
  auto mesh_layer_G_ptr =
      std::make_shared<MeshLayer>(tsdf_submap_collection_ptr_->block_size());
  transformMeshLayerToGlobalFrame(*active_submap_mesh_layer_ptr_,
                                  mesh_layer_G_ptr.get());
  // Coloring the mesh
  colorMeshWithCurrentIndex(mesh_layer_G_ptr.get());
  return mesh_layer_G_ptr;
}

}  // namespace cbloxa