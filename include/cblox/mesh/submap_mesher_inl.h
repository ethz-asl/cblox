//
// Created by victor on 17.01.19.
//

#ifndef CBLOX_MESH_SUBMAP_MESHER_INL_H_
#define CBLOX_MESH_SUBMAP_MESHER_INL_H_

#include <vector>

namespace cblox {
template <typename SubmapType>
void SubmapMesher::generateSeparatedMesh(
    const SubmapCollection<SubmapType>& submap_collection,
    MeshLayer* seperated_mesh_layer_ptr) {
  // Checks
  CHECK_NOTNULL(seperated_mesh_layer_ptr);
  // Getting the submaps
  const std::vector<typename SubmapType::Ptr> sub_maps =
      submap_collection.getSubMaps();
  // Generating the mesh layers
  std::vector<MeshLayer::Ptr> sub_map_mesh_layers;
  generateSeparatedMeshLayers<SubmapType>(sub_maps, &sub_map_mesh_layers);

  // Coloring the mesh layers
  colorMeshLayersWithIndex(&sub_map_mesh_layers);
  // Get submap transforms
  AlignedVector<Transformation> sub_map_poses;
  submap_collection.getSubMapPoses(&sub_map_poses);
  // Combining the mesh layers
  combineMeshLayers(sub_map_mesh_layers, sub_map_poses,
                    seperated_mesh_layer_ptr);
}

template <typename SubmapType>
void SubmapMesher::generateCombinedMesh(
    const SubmapCollection<SubmapType>& submap_collection,
    MeshLayer* combined_mesh_layer_ptr) {
  // Checks
  CHECK_NOTNULL(combined_mesh_layer_ptr);
  // Getting the submaps
  const std::vector<TsdfSubmap::Ptr> tsdf_sub_maps =
      submap_collection.getSubMaps();
  // Getting the Tsdf map which is the projection of the submap collection
  TsdfMap::Ptr combined_tsdf_map_ptr = submap_collection.getProjectedMap();
  // Creating a new mesh layer and making it active
  MeshIntegrator<TsdfVoxel> mesh_integrator(
      mesh_config_, combined_tsdf_map_ptr->getTsdfLayerPtr(),
      combined_mesh_layer_ptr);
  // Generating the mesh
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);
}

template <typename SubmapType>
void SubmapMesher::generateSeparatedMeshLayers(
    const std::vector<typename SubmapType::Ptr>& sub_maps,
    std::vector<MeshLayer::Ptr>* sub_map_mesh_layers) {
  // Checks
  CHECK_NOTNULL(sub_map_mesh_layers);
  sub_map_mesh_layers->clear();
  sub_map_mesh_layers->reserve(sub_maps.size());
  // Looping over the sub maps and generating meshs
  size_t mesh_index = 0;
  for (typename SubmapType::Ptr sub_map_ptr : sub_maps) {
    CHECK_NOTNULL(sub_map_ptr.get());
    // DEBUG
    std::cout << "Generating mesh for submap number #" << mesh_index
              << std::endl;
    mesh_index++;
    // Getting the TSDF data
    TsdfMap::Ptr tsdf_map_ptr = sub_map_ptr->getTsdfMapPtr();
    // Creating a mesh layer to hold the mesh fragment
    MeshLayer::Ptr mesh_layer_ptr(
        new MeshLayer(sub_map_ptr->getTsdfMap().block_size()));
    // Generating the mesh
    MeshIntegrator<TsdfVoxel> mesh_integrator(
        mesh_config_, tsdf_map_ptr->getTsdfLayerPtr(), mesh_layer_ptr.get());
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);
    // Pushing this mesh layer to the output
    sub_map_mesh_layers->push_back(mesh_layer_ptr);
  }
}
}  // namespace cblox

#endif  // CBLOX_MESH_SUBMAP_MESHER_INL_H_
