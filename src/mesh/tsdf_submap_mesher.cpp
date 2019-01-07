#include <iostream>
#include <vector>

#include <glog/logging.h>

#include "cblox/mesh/tsdf_submap_mesher.h"

namespace cblox {

using voxblox::IndexElement;
using voxblox::BlockIndex;
using voxblox::BlockIndexList;
using voxblox::Point;

void TsdfSubmapMesher::generateSeparatedMesh(
    const TsdfSubmapCollection& tsdf_submap_collection,
    MeshLayer* seperated_mesh_layer_ptr) {
  // Checks
  CHECK_NOTNULL(seperated_mesh_layer_ptr);
  // Getting the submaps
  const std::vector<TsdfSubmap::Ptr> tsdf_sub_maps =
      tsdf_submap_collection.getSubMaps();
  // Generating the mesh layers
  std::vector<MeshLayer::Ptr> sub_map_mesh_layers;
  generateSeparatedMeshLayers(tsdf_sub_maps, &sub_map_mesh_layers);

  // Coloring the mesh layers
  colorMeshLayersWithIndex(&sub_map_mesh_layers);
  // Get submap transforms
  AlignedVector<Transformation> sub_map_poses;
  tsdf_submap_collection.getSubMapPoses(&sub_map_poses);
  // Combining the mesh layers
  combineMeshLayers(sub_map_mesh_layers, sub_map_poses,
                    seperated_mesh_layer_ptr);
}

void TsdfSubmapMesher::generateCombinedMesh(
    const TsdfSubmapCollection& tsdf_submap_collection,
    MeshLayer* combined_mesh_layer_ptr) {
  // Checks
  CHECK_NOTNULL(combined_mesh_layer_ptr);
  // Getting the submaps
  const std::vector<TsdfSubmap::Ptr> tsdf_sub_maps =
      tsdf_submap_collection.getSubMaps();
  // Getting the Tsdf map which is the projection of the submap collection
  TsdfMap::Ptr combined_tsdf_map_ptr = tsdf_submap_collection.getProjectedMap();
  // Creating a new mesh layer and making it active
  MeshIntegrator<TsdfVoxel> mesh_integrator(
      mesh_config_, combined_tsdf_map_ptr->getTsdfLayerPtr(),
      combined_mesh_layer_ptr);
  // Generating the mesh
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);
}

void TsdfSubmapMesher::generatePatchMeshes(
    const TsdfSubmapCollection& tsdf_submap_collection,
    std::vector<MeshLayer::Ptr>* sub_map_mesh_layers_ptr) {
  // Checks
  CHECK_NOTNULL(sub_map_mesh_layers_ptr);
  // Getting the submaps
  const std::vector<TsdfSubmap::Ptr> tsdf_sub_maps =
      tsdf_submap_collection.getSubMaps();
  // Generating the mesh layers
  generateSeparatedMeshLayers(tsdf_sub_maps, sub_map_mesh_layers_ptr);
}

void TsdfSubmapMesher::generateInterpolationTestMesh(
    const TsdfSubmapCollection& tsdf_submap_collection,
    MeshLayer* interpolation_test_mesh_layer_ptr) {
  // Checks
  CHECK_NOTNULL(interpolation_test_mesh_layer_ptr);

  // Getting the block size
  double block_size = tsdf_submap_collection.block_size();
  // Current number of sub maps
  size_t num_sub_maps = tsdf_submap_collection.size();

  // Getting the submaps
  const std::vector<TsdfSubmap::Ptr> tsdf_sub_maps =
      tsdf_submap_collection.getSubMaps();

  // A vector containing the transformed tsdf layers
  std::vector<TsdfMap::Ptr> transformed_sub_maps;
  transformed_sub_maps.reserve(num_sub_maps);
  for (TsdfSubmap::ConstPtr tsdf_sub_map_ptr : tsdf_sub_maps) {
    // Creating a new map in the world frame
    TsdfMap::Ptr transformed_sub_map_ptr(new TsdfMap(tsdf_map_config_));
    Layer<TsdfVoxel>* transformed_sub_map_tsdf_layer_ptr =
        transformed_sub_map_ptr->getTsdfLayerPtr();
    // Getting the tsdf submap and its pose
    const TsdfMap& tsdf_map = tsdf_sub_map_ptr->getTsdfMap();
    const Transformation& T_M_S = tsdf_sub_map_ptr->getPose();
    // Merging layers the submap into the global layer
    mergeLayerAintoLayerB(tsdf_map.getTsdfLayer(), T_M_S,
                          transformed_sub_map_tsdf_layer_ptr);
    // Pushing this onto the list of transformed sub maps
    transformed_sub_maps.push_back(transformed_sub_map_ptr);
  }

  // Creating a vector of the meshs from the transformed layers
  std::vector<MeshLayer> mesh_layers;
  mesh_layers.reserve(num_sub_maps);
  for (TsdfMap::Ptr transformed_sub_map_ptr : transformed_sub_maps) {
    // Creating a new meshlayer
    MeshLayer mesh_layer(block_size);
    // Creating a new mesh layer and making it active
    MeshIntegrator<TsdfVoxel> mesh_integrator(
        mesh_config_, transformed_sub_map_ptr->getTsdfLayerPtr(), &mesh_layer);
    // Generating the mesh
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);
    // Pushing this onto the list
    mesh_layers.push_back(mesh_layer);
  }

  // Looping over submaps and coloring the meshes
  for (size_t sub_map_index = 0; sub_map_index < num_sub_maps;
       sub_map_index++) {
    // Extracting the mesh layer
    MeshLayer mesh_layer = mesh_layers[sub_map_index];
    // Generating a color
    double color_map_index = static_cast<double>(sub_map_index) /
                             static_cast<double>(num_sub_maps - 1);
    Color sub_map_color = voxblox::rainbowColorMap(color_map_index);
    // Coloring this mesh layer
    colorMeshLayer(sub_map_color, &mesh_layer);
  }

  // Combining the mesh layers
  MeshLayer::Ptr combined_mesh_layer_ptr(new MeshLayer(block_size));
  for (const MeshLayer& mesh_layer : mesh_layers) {
    // Looping over all the blocks on this layer
    BlockIndexList block_index_list;
    mesh_layer.getAllAllocatedMeshes(&block_index_list);
    for (const BlockIndex& block_index : block_index_list) {
      // Getting mesh in this submap block
      Mesh::ConstPtr mesh_ptr = mesh_layer.getMeshPtrByIndex(block_index);
      // Loop over points and adding to the global mesh. Incrementing in
      // triplets
      size_t num_vertices = mesh_ptr->vertices.size();
      if (num_vertices > 0) {
        for (VertexIndex start_index = 0; start_index < num_vertices - 2;
             start_index += 3) {
          // Transforming the first triangle vertex to the world frame
          Point vertex_1_M = mesh_ptr->vertices[start_index];
          // Getting the mesh at the location of the first vertex of the
          // triangle
          Mesh::Ptr mesh_combined_ptr =
              combined_mesh_layer_ptr->allocateMeshPtrByCoordinates(vertex_1_M);
          // Transforming the triangle and adding it to the combined mesh
          addTriangleToMesh(*mesh_ptr, start_index, mesh_combined_ptr.get());
        }
      }
    }
  }
}

void TsdfSubmapMesher::generateTrimmedCombinedMesh(
    const TsdfSubmapCollection& tsdf_submap_collection, double mesh_trim_height,
    MeshLayer* combined_mesh_layer_ptr) {
  // Checks
  CHECK_NOTNULL(combined_mesh_layer_ptr);
  std::cout << "Starting map trimming." << std::endl;
  // Getting the submaps
  const std::vector<TsdfSubmap::Ptr> tsdf_sub_maps =
      tsdf_submap_collection.getSubMaps();
  // The new trimmed TSDF submaps
  std::vector<TsdfSubmap::Ptr> trimmed_tsdf_sub_maps;
  trimmed_tsdf_sub_maps.reserve(tsdf_sub_maps.size());
  // Looping over the current submaps and trimming
  for (TsdfSubmap::ConstPtr tsdf_sub_map_ptr : tsdf_sub_maps) {
    // Creating a new map
    TsdfSubmap::Ptr trimmed_tsdf_sub_map_ptr(
        new TsdfSubmap(tsdf_sub_map_ptr->getPose(), tsdf_sub_map_ptr->getID(),
                       tsdf_map_config_));
    // Trimming the map
    trimSubmapToHeight(mesh_trim_height, *tsdf_sub_map_ptr,
                       trimmed_tsdf_sub_map_ptr.get());
    // Adding the new map to the list
    trimmed_tsdf_sub_maps.push_back(trimmed_tsdf_sub_map_ptr);
  }
  // Creating a new submap collection from these maps
  TsdfSubmapCollection tsdf_submap_collection_trimmed(
      tsdf_submap_collection.getConfig(), trimmed_tsdf_sub_maps);
  std::cout << "Starting combination." << std::endl;
  // Getting the Tsdf map which is the projection of the submap collection
  TsdfMap::Ptr combined_tsdf_map_ptr =
      tsdf_submap_collection_trimmed.getProjectedMap();
  // Getting the pointers to the combined Tsdf Map
  // MeshLayer combined_mesh_layer(combined_tsdf_map_ptr->block_size());
  // Creating a new mesh layer and making it active
  MeshIntegrator<TsdfVoxel> mesh_integrator(
      mesh_config_, combined_tsdf_map_ptr->getTsdfLayerPtr(),
      combined_mesh_layer_ptr);
  // Generating the mesh
  std::cout << "Starting meshing." << std::endl;
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);
}

void TsdfSubmapMesher::generateSeparatedMeshLayers(
    const std::vector<TsdfSubmap::Ptr>& tsdf_sub_maps,
    std::vector<MeshLayer::Ptr>* sub_map_mesh_layers) {
  // Checks
  CHECK_NOTNULL(sub_map_mesh_layers);
  sub_map_mesh_layers->clear();
  sub_map_mesh_layers->reserve(tsdf_sub_maps.size());
  // Looping over the sub maps and generating meshs
  size_t mesh_index = 0;
  for (TsdfSubmap::Ptr tsdf_sub_map_ptr : tsdf_sub_maps) {
    CHECK_NOTNULL(tsdf_sub_map_ptr.get());
    // DEBUG
    std::cout << "Generating mesh for submap number #" << mesh_index
              << std::endl;
    mesh_index++;
    // Getting the TSDF data
    TsdfMap::Ptr tsdf_map_ptr = tsdf_sub_map_ptr->getTsdfMapPtr();
    // Creating a mesh layer to hold the mesh fragment
    MeshLayer::Ptr mesh_layer_ptr(
        new MeshLayer(tsdf_sub_map_ptr->getTsdfMap().block_size()));
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

void TsdfSubmapMesher::combineMeshLayers(
    const std::vector<MeshLayer::Ptr>& sub_map_mesh_layers,
    const AlignedVector<Transformation>& sub_map_poses,
    MeshLayer* combined_mesh_layer_ptr) {
  // Checks
  CHECK_NOTNULL(combined_mesh_layer_ptr);
  // DEBUG
  std::cout << "Starting combining mesh layers." << std::endl;
  // Looping over all the mesh layers for the submaps
  for (size_t sub_map_index = 0; sub_map_index < sub_map_mesh_layers.size();
       sub_map_index++) {
    // Getting the mesh layer and the transform
    Transformation T_M_S = sub_map_poses[sub_map_index];
    MeshLayer::Ptr mesh_layer_ptr = sub_map_mesh_layers[sub_map_index];
    // Transforming all triangles in the mesh and adding to the combined layer
    transformAndAddTrianglesToLayer(*mesh_layer_ptr, T_M_S,
                                    combined_mesh_layer_ptr);
  }
}

void TsdfSubmapMesher::transformMeshLayers(
    const std::vector<MeshLayer::Ptr>& sub_map_mesh_layers,
    const AlignedVector<Transformation>& sub_map_poses,
    std::vector<MeshLayer::Ptr>* transformed_sub_map_mesh_layers) {
  // Checks
  CHECK_NOTNULL(transformed_sub_map_mesh_layers);
  transformed_sub_map_mesh_layers->clear();
  transformed_sub_map_mesh_layers->reserve(sub_map_mesh_layers.size());
  // DEBUG
  std::cout << "Starting Transforming mesh layers." << std::endl;
  // Looping over all the mesh layers for the submaps
  for (size_t sub_map_index = 0; sub_map_index < sub_map_mesh_layers.size();
       sub_map_index++) {
    // Getting the mesh layer and the transform
    Transformation T_M_S = sub_map_poses[sub_map_index];
    MeshLayer::Ptr mesh_layer_ptr = sub_map_mesh_layers[sub_map_index];
    // Making a new layer for the transformed triangles
    MeshLayer::Ptr transformed_mesh_layer_ptr =
        transformMeshLayer(*mesh_layer_ptr, T_M_S);
    // Adding to the mesh layer vector
    transformed_sub_map_mesh_layers->push_back(transformed_mesh_layer_ptr);
  }
}

MeshLayer::Ptr TsdfSubmapMesher::transformMeshLayer(
    const MeshLayer& mesh_layer, const Transformation& transformation) {
  // The ptr holding the transformed mesh layer
  MeshLayer::Ptr transformed_mesh_layer_ptr(
      new MeshLayer(mesh_layer.block_size()));
  // Transforming all triangles in the mesh and adding to the output vector
  transformAndAddTrianglesToLayer(mesh_layer, transformation,
                                  transformed_mesh_layer_ptr.get());
  // Returning the mesh layer
  return transformed_mesh_layer_ptr;
}

void TsdfSubmapMesher::colorMeshLayersWithIndex(
    std::vector<MeshLayer::Ptr>* sub_map_mesh_layers) const {
  // Looping over submaps and coloring
  size_t num_sub_maps = sub_map_mesh_layers->size();
  for (size_t sub_map_index = 0; sub_map_index < num_sub_maps;
       sub_map_index++) {
    // Extracting the mesh layer
    MeshLayer* mesh_layer_ptr = ((*sub_map_mesh_layers)[sub_map_index]).get();
    // Generating a color
    double color_map_index = static_cast<double>(sub_map_index) /
                             static_cast<double>(num_sub_maps - 1);
    Color sub_map_color = voxblox::rainbowColorMap(color_map_index);
    // Coloring this mesh layer
    colorMeshLayer(sub_map_color, mesh_layer_ptr);
  }
}

void TsdfSubmapMesher::colorMeshLayer(const Color& color_in,
                                      MeshLayer* mesh_layer_ptr) const {
  // Looping over all the blocks on this layer
  BlockIndexList meshes_block_index_list;
  mesh_layer_ptr->getAllAllocatedMeshes(&meshes_block_index_list);
  for (const BlockIndex& block_index : meshes_block_index_list) {
    // Getting the mesh in this block
    Mesh::Ptr mesh_ptr = mesh_layer_ptr->getMeshPtrByIndex(block_index);
    // Coloring the vertices in the mesh mesh
    for (Color& color : mesh_ptr->colors) {
      color = color_in;
    }
  }
}

void TsdfSubmapMesher::transformAndAddTrianglesToLayer(
    const MeshLayer& input_mesh_layer, const Transformation& T_B_A,
    MeshLayer* output_mesh_layer) {
  // Checks
  CHECK_NOTNULL(output_mesh_layer);
  // Looping over all the blocks on this layer
  BlockIndexList block_index_list;
  input_mesh_layer.getAllAllocatedMeshes(&block_index_list);

  for (const BlockIndex& block_index : block_index_list) {
    // Getting mesh in this submap block
    Mesh mesh = input_mesh_layer.getMeshByIndex(block_index);
    // Loop over points and adding to the global mesh. Incrementing in
    // triplets
    size_t num_vertices = mesh.vertices.size();
    if (num_vertices > 0) {
      for (VertexIndex start_index = 0; start_index < num_vertices - 2;
           start_index = start_index + 3) {
        // Transforming the first triangle vertex to the world frame
        Point vertex_1_M = T_B_A * mesh.vertices[start_index];
        // Getting the mesh at the location of the first vertex of the
        // triangle
        Mesh::Ptr mesh_combined_ptr =
            output_mesh_layer->allocateMeshPtrByCoordinates(vertex_1_M);
        // Transforming the triangle and adding it to the combined mesh
        transformAndAddTriangleToMesh(mesh, start_index, T_B_A,
                                      mesh_combined_ptr.get());
      }
    }
  }
}

void TsdfSubmapMesher::transformAndAddTriangleToMesh(
    const Mesh& input_mesh, const VertexIndex start_index,
    const Transformation& T_B_A, Mesh* output_mesh) {
  // CHECKS
  CHECK_NOTNULL(output_mesh);
  CHECK_LT(start_index, input_mesh.vertices.size() - 2);
  // Copying in the triangle
  Point vertex_1_M = T_B_A * input_mesh.vertices[start_index];
  Point vertex_2_M = T_B_A * input_mesh.vertices[start_index + 1];
  Point vertex_3_M = T_B_A * input_mesh.vertices[start_index + 2];
  // Adding the meshs points to this block
  output_mesh->vertices.push_back(vertex_1_M);
  output_mesh->vertices.push_back(vertex_2_M);
  output_mesh->vertices.push_back(vertex_3_M);
  if (input_mesh.hasNormals()) {
    output_mesh->normals.push_back(input_mesh.normals[start_index]);
    output_mesh->normals.push_back(input_mesh.normals[start_index + 1]);
    output_mesh->normals.push_back(input_mesh.normals[start_index + 2]);
  }
  if (input_mesh.hasColors()) {
    output_mesh->colors.push_back(input_mesh.colors[start_index]);
    output_mesh->colors.push_back(input_mesh.colors[start_index + 1]);
    output_mesh->colors.push_back(input_mesh.colors[start_index + 2]);
  }
  if (input_mesh.hasTriangles()) {
    // size_t current_max_index =
    // *std::max_element(output_mesh->indices.begin(),
    //                                             output_mesh->indices.end());
    VertexIndex current_max_index = output_mesh->indices.size();
    output_mesh->indices.push_back(current_max_index + 0);
    output_mesh->indices.push_back(current_max_index + 1);
    output_mesh->indices.push_back(current_max_index + 2);
  }
}

void TsdfSubmapMesher::addTrianglesToLayer(const MeshLayer& input_mesh_layer,
                                           MeshLayer* output_mesh_layer) {
  // Checks
  CHECK_NOTNULL(output_mesh_layer);
  // Looping over all the blocks on this layer
  BlockIndexList block_index_list;
  input_mesh_layer.getAllAllocatedMeshes(&block_index_list);
  for (const BlockIndex& block_index : block_index_list) {
    // Getting mesh in this submap block
    Mesh mesh = input_mesh_layer.getMeshByIndex(block_index);
    // Loop over points and adding to the global mesh. Incrementing in
    // triplets
    size_t num_vertices = mesh.vertices.size();
    if (num_vertices > 0) {
      for (VertexIndex start_index = 0; start_index < num_vertices - 2;
           start_index = start_index + 3) {
        // Getting the mesh at the location of the first vertex of the
        // triangle
        Mesh::Ptr mesh_combined_ptr =
            output_mesh_layer->allocateMeshPtrByCoordinates(
                mesh.vertices[start_index]);
        // Transforming the triangle and adding it to the combined mesh
        addTriangleToMesh(mesh, start_index, mesh_combined_ptr.get());
      }
    }
  }
}

void TsdfSubmapMesher::addTriangleToMesh(const Mesh& input_mesh,
                                         const VertexIndex start_index,
                                         Mesh* output_mesh) {
  // CHECKS
  CHECK_NOTNULL(output_mesh);
  CHECK_LT(start_index, input_mesh.vertices.size() - 2);
  // Copying in the triangle
  Point vertex_1_M = input_mesh.vertices[start_index];
  Point vertex_2_M = input_mesh.vertices[start_index + 1];
  Point vertex_3_M = input_mesh.vertices[start_index + 2];
  // Adding the meshs points to this block
  output_mesh->vertices.push_back(vertex_1_M);
  output_mesh->vertices.push_back(vertex_2_M);
  output_mesh->vertices.push_back(vertex_3_M);
  if (input_mesh.hasNormals()) {
    output_mesh->normals.push_back(input_mesh.normals[start_index]);
    output_mesh->normals.push_back(input_mesh.normals[start_index + 1]);
    output_mesh->normals.push_back(input_mesh.normals[start_index + 2]);
  }
  if (input_mesh.hasColors()) {
    output_mesh->colors.push_back(input_mesh.colors[start_index]);
    output_mesh->colors.push_back(input_mesh.colors[start_index + 1]);
    output_mesh->colors.push_back(input_mesh.colors[start_index + 2]);
  }
  if (input_mesh.hasTriangles()) {
    //    size_t current_max_index =
    //    *std::max_element(output_mesh->indices.begin(),
    //                                                 output_mesh->indices.end());
    VertexIndex current_max_index = output_mesh->indices.size();
    output_mesh->indices.push_back(current_max_index + 0);
    output_mesh->indices.push_back(current_max_index + 1);
    output_mesh->indices.push_back(current_max_index + 2);
  }
}

void TsdfSubmapMesher::trimSubmapToHeight(
    const float trim_height, const TsdfSubmap& tsdf_sub_map,
    TsdfSubmap* trimmed_tsdf_sub_map_ptr) const {
  // Checks
  CHECK_NOTNULL(trimmed_tsdf_sub_map_ptr);
  // Getting the Tsdf layer from the trimmed map
  TsdfMap::Ptr trimmed_tsdf_map_ptr = trimmed_tsdf_sub_map_ptr->getTsdfMapPtr();
  Layer<TsdfVoxel>* trimmed_tsdf_layer_ptr =
      trimmed_tsdf_map_ptr->getTsdfLayerPtr();
  // Looping over the original map and adding voxel which are under the height
  const TsdfMap& tsdf_map = tsdf_sub_map.getTsdfMap();
  const Layer<TsdfVoxel>& tsdf_layer = tsdf_map.getTsdfLayer();
  BlockIndexList block_index_list;
  tsdf_layer.getAllAllocatedBlocks(&block_index_list);
  for (const BlockIndex& block_index : block_index_list) {
    // Allocating a block here in the new map
    Block<TsdfVoxel>::Ptr trimmed_block_ptr =
        trimmed_tsdf_layer_ptr->allocateBlockPtrByIndex(block_index);

    // Extracting the block
    const Block<TsdfVoxel>& block = tsdf_layer.getBlockByIndex(block_index);
    // Looping over the voxels in the block
    for (IndexElement voxel_index = 0; voxel_index < block.num_voxels();
         ++voxel_index) {
      // Getting the voxel's coordinates in the submap frame
      Point t_S_V = block.computeCoordinatesFromLinearIndex(voxel_index);
      // If this voxel is sufficiently low, adding it to the new submap
      // NOTE(alex.millane): The submap base frames are defined as the first
      //                     CAMERA pose in the map. Therefore positive y
      //                     represent "down" (in a gravity sense).
      if (t_S_V[1] >= -trim_height) {
        // Original voxel
        const TsdfVoxel& voxel = block.getVoxelByLinearIndex(voxel_index);
        // Voxel in trimmed map
        TsdfVoxel& trimmed_voxel =
            trimmed_block_ptr->getVoxelByLinearIndex(voxel_index);
        // Copying voxel to trimmed map
        trimmed_voxel = voxel;
        // Indicating that this block has data
        trimmed_block_ptr->has_data() = true;
      }
    }
    // If no data was added here, remove this block
    if (!trimmed_block_ptr->has_data()) {
      trimmed_tsdf_layer_ptr->removeBlock(block_index);
    }
  }
}

}  // namespace cblox
