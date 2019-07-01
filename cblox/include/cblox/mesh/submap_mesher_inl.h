#ifndef CBLOX_MESH_SUBMAP_MESHER_INL_H_
#define CBLOX_MESH_SUBMAP_MESHER_INL_H_

#include <vector>

namespace cblox {
template <typename SubmapType>
void SubmapMesher::generateSeparatedMesh(
    const SubmapCollection<SubmapType>& submap_collection,
    MeshLayer* seperated_mesh_layer_ptr) {
  CHECK_NOTNULL(seperated_mesh_layer_ptr);
  // Getting the submaps
  const std::vector<typename SubmapType::ConstPtr> sub_maps =
      submap_collection.getSubmapConstPtrs();
  // Generating the mesh layers
  std::vector<MeshLayer::Ptr> sub_map_mesh_layers;
  generateSeparatedMeshLayers<SubmapType>(sub_maps, &sub_map_mesh_layers);
  // Coloring the mesh layers
  colorMeshLayersWithIndex(&sub_map_mesh_layers);
  // Get submap transforms
  AlignedVector<Transformation> sub_map_poses;
  submap_collection.getSubmapPoses(&sub_map_poses);
  // Combining the mesh layers
  // NOTE(alexmillane): Have to construct a vector of pointers to const...
  combineMeshLayers(std::vector<MeshLayer::ConstPtr>(
                        sub_map_mesh_layers.begin(), sub_map_mesh_layers.end()),
                    sub_map_poses, seperated_mesh_layer_ptr);
}

template <typename SubmapType>
void SubmapMesher::generateCombinedMesh(
    const SubmapCollection<SubmapType>& submap_collection,
    MeshLayer* combined_mesh_layer_ptr) {
  CHECK_NOTNULL(combined_mesh_layer_ptr);
  // Getting the Tsdf map which is the projection of the submap collection
  TsdfMap::Ptr combined_tsdf_map_ptr = submap_collection.getProjectedMap();
  // Creating a new mesh layer and making it active
  MeshIntegrator<TsdfVoxel> mesh_integrator(
      mesh_config_, combined_tsdf_map_ptr->getTsdfLayer(),
      combined_mesh_layer_ptr);
  // Generating the mesh
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = false;
  mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);
}

template <typename SubmapType>
void SubmapMesher::generateSeparatedMeshLayers(
    const std::vector<typename SubmapType::ConstPtr>& sub_maps,
    std::vector<MeshLayer::Ptr>* sub_map_mesh_layers) {
  CHECK_NOTNULL(sub_map_mesh_layers);
  sub_map_mesh_layers->clear();
  sub_map_mesh_layers->reserve(sub_maps.size());
  // Looping over the sub maps and generating meshs
  size_t mesh_index = 0;
  for (typename SubmapType::ConstPtr sub_map_ptr : sub_maps) {
    CHECK_NOTNULL(sub_map_ptr.get());
    LOG(INFO) << "Generating mesh for submap number #" << mesh_index;
    mesh_index++;
    // Getting the TSDF data
    const TsdfMap& tsdf_map = sub_map_ptr->getTsdfMap();
    // Creating a mesh layer to hold the mesh fragment
    MeshLayer::Ptr mesh_layer_ptr(
        new MeshLayer(sub_map_ptr->getTsdfMap().block_size()));
    // Generating the mesh
    MeshIntegrator<TsdfVoxel> mesh_integrator(
        mesh_config_, tsdf_map.getTsdfLayer(), mesh_layer_ptr.get());
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = false;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);
    // Pushing this mesh layer to the output
    sub_map_mesh_layers->push_back(mesh_layer_ptr);
  }
}

template <typename SubmapType>
void SubmapMesher::generateMeshInGlobalFrame(const SubmapType& submap,
                                             MeshLayer* mesh_layer_G_ptr) {
  CHECK_NOTNULL(mesh_layer_G_ptr);
  // Mesh in submap frame
  MeshLayer mesh_layer_S(submap.getTsdfMap().block_size());
  generateMeshInSubmapFrame(submap, &mesh_layer_S);
  // To world frame
  const Transformation& T_G_S = submap.getPose();
  transformAndAddTrianglesToLayer(mesh_layer_S, T_G_S, mesh_layer_G_ptr);
}

template <typename SubmapType>
void SubmapMesher::generateMeshInSubmapFrame(const SubmapType& submap,
                                             MeshLayer* mesh_layer_S_ptr) {
  CHECK_NOTNULL(mesh_layer_S_ptr);
  // Getting the TSDF data
  const TsdfMap& tsdf_map = submap.getTsdfMap();
  // Generating the mesh
  MeshIntegrator<TsdfVoxel> mesh_integrator(
      mesh_config_, tsdf_map.getTsdfLayer(), mesh_layer_S_ptr);
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = false;
  mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);
}

}  // namespace cblox

#endif  // CBLOX_MESH_SUBMAP_MESHER_INL_H_
