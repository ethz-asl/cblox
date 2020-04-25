#include "cblox_ros/active_submap_visualizer.h"

#include <cblox/mesh/submap_mesher.h>

namespace cblox {

void ActiveSubmapVisualizer::switchToSubmap(const SubmapID submap_id) {
  CHECK(tsdf_submap_collection_ptr_);
  active_submap_id_ = submap_id;
  if (mesh_layers_.find(submap_id) == mesh_layers_.end()) {
    if (verbose_) {
      ROS_INFO_STREAM("Creating mesh layer for submap id: " << submap_id);
    }
    createMeshLayer();
    updateIntegrator();
  } else {
    if (verbose_) {
      ROS_INFO_STREAM("Recovering mesh layer for submap id: " << submap_id);
    }
    recoverMeshLayer();
    updateIntegrator();
  }
}

void ActiveSubmapVisualizer::switchToActiveSubmap() {
  // Getting the active submap ID
  const SubmapID submap_id = tsdf_submap_collection_ptr_->getActiveSubmapID();
  switchToSubmap(submap_id);
}

void ActiveSubmapVisualizer::createMeshLayer() {
  // Active layer stuff
  CHECK(tsdf_submap_collection_ptr_);
  active_submap_mesh_layer_ptr_.reset(
      new voxblox::MeshLayer(tsdf_submap_collection_ptr_->block_size()));
  active_submap_color_idx_ = current_color_idx_;
  // Saving mesh layer and color for later recovery
  mesh_layers_[active_submap_id_] = active_submap_mesh_layer_ptr_;
  mesh_color_indices_[active_submap_id_] = active_submap_color_idx_;
  // Updating the color index for the next map.
  current_color_idx_ = (current_color_idx_ + 1) % color_cycle_length_;
}

void ActiveSubmapVisualizer::recoverMeshLayer() {
  auto mesh_it = mesh_layers_.find(active_submap_id_);
  auto color_it = mesh_color_indices_.find(active_submap_id_);
  CHECK(mesh_it != mesh_layers_.end())
      << "Tried to recover layer not already created";
  CHECK(color_it != mesh_color_indices_.end())
      << "Tried to recover layer not already created";
  active_submap_mesh_layer_ptr_ = mesh_it->second;
  active_submap_color_idx_ = color_it->second;
}

void ActiveSubmapVisualizer::updateIntegrator() {
  CHECK(active_submap_mesh_layer_ptr_) << "MeshLayer not initialized.";
  // New integrator operating on the mesh.
  active_submap_mesh_integrator_ptr_.reset(
      new voxblox::MeshIntegrator<TsdfVoxel>(
          mesh_config_,
          tsdf_submap_collection_ptr_->getTsdfMapPtr(active_submap_id_)
              ->getTsdfLayerPtr(),
          active_submap_mesh_layer_ptr_.get()));
}

void ActiveSubmapVisualizer::updateMeshLayer() {
  CHECK(active_submap_mesh_integrator_ptr_) << "Integrator not initialized.";
  // Updating the mesh layer
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  active_submap_mesh_integrator_ptr_->generateMesh(only_mesh_updated_blocks,
                                                   clear_updated_flag);
}

void ActiveSubmapVisualizer::transformMeshLayerToGlobalFrame(
    const MeshLayer& mesh_layer_S, MeshLayer* mesh_layer_G_ptr) const {
  CHECK_NOTNULL(mesh_layer_G_ptr);
  // Transforming all triangles in the mesh and adding to the combined layer
  Transformation T_G_S;
  tsdf_submap_collection_ptr_->getSubmapPose(active_submap_id_, &T_G_S);
  SubmapMesher::transformAndAddTrianglesToLayer(mesh_layer_S, T_G_S,
                                                mesh_layer_G_ptr);
}

void ActiveSubmapVisualizer::colorMeshWithCurrentIndex(
    MeshLayer* mesh_layer_ptr) const {
  CHECK_NOTNULL(mesh_layer_ptr);
  // Coloring
  double color_map_float = static_cast<double>(active_submap_color_idx_) /
                           static_cast<double>(color_cycle_length_ - 1);
  const Color color = voxblox::rainbowColorMap(color_map_float);
  SubmapMesher::colorMeshLayer(color, mesh_layer_ptr);
}

MeshLayer::Ptr ActiveSubmapVisualizer::getDisplayMeshLayer() {
  // Transforming the mesh layer into G.
  auto mesh_layer_G_ptr =
      std::make_shared<MeshLayer>(tsdf_submap_collection_ptr_->block_size());
  transformMeshLayerToGlobalFrame(*active_submap_mesh_layer_ptr_,
                                  mesh_layer_G_ptr.get());
  // Coloring the mesh
  colorMeshWithCurrentIndex(mesh_layer_G_ptr.get());
  return mesh_layer_G_ptr;
}

void ActiveSubmapVisualizer::getDisplayMesh(
    visualization_msgs::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  // Getting the mesh layer
  std::shared_ptr<MeshLayer> mesh_layer_ptr = getDisplayMeshLayer();
  // Filling the marker
  const voxblox::ColorMode color_mode = voxblox::ColorMode::kLambertColor;
  voxblox::fillMarkerWithMesh(mesh_layer_ptr, color_mode, marker_ptr);
  marker_ptr->id = active_submap_id_;
  // Setting opacity of marker
  marker_ptr->color.a = opacity_;
  for (std_msgs::ColorRGBA& color : marker_ptr->colors) {
    color.a = opacity_;
  }
}

}  // namespace cblox
