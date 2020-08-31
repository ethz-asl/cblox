#include "cblox/mesh/submap_mesher.h"

#include <vector>

#include <glog/logging.h>

namespace cblox {

using voxblox::BlockIndex;
using voxblox::BlockIndexList;
using voxblox::IndexElement;
using voxblox::Point;

void SubmapMesher::generatePatchMeshes(
    const SubmapCollection<TsdfSubmap>& tsdf_submap_collection,
    std::vector<MeshLayer::Ptr>* sub_map_mesh_layers_ptr) {
  CHECK_NOTNULL(sub_map_mesh_layers_ptr);
  // Getting the submaps
  const std::vector<TsdfSubmap::ConstPtr> tsdf_sub_maps =
      tsdf_submap_collection.getSubmapConstPtrs();
  // Generating the mesh layers
  generateSeparatedMeshLayers<TsdfSubmap>(tsdf_sub_maps,
                                          sub_map_mesh_layers_ptr);
}

void SubmapMesher::combineMeshLayers(
    const std::vector<MeshLayer::ConstPtr>& sub_map_mesh_layers,
    const AlignedVector<Transformation>& sub_map_poses,
    MeshLayer* combined_mesh_layer_ptr) {
  CHECK_NOTNULL(combined_mesh_layer_ptr);
  // Looping over all the mesh layers for the submaps
  LOG(INFO) << "Starting combining mesh layers.";
  for (size_t sub_map_index = 0; sub_map_index < sub_map_mesh_layers.size();
       sub_map_index++) {
    LOG(INFO) << "Adding layer: " << sub_map_index;
    // Getting the mesh layer and the transform
    const Transformation& T_M_S = sub_map_poses[sub_map_index];
    MeshLayer::ConstPtr mesh_layer_ptr = sub_map_mesh_layers[sub_map_index];
    // Transforming all triangles in the mesh and adding to the combined layer
    transformAndAddTrianglesToLayer(*mesh_layer_ptr, T_M_S,
                                    combined_mesh_layer_ptr);
  }
}

void SubmapMesher::transformMeshLayers(
    const std::vector<MeshLayer::ConstPtr>& sub_map_mesh_layers,
    const AlignedVector<Transformation>& sub_map_poses,
    std::vector<MeshLayer::Ptr>* transformed_sub_map_mesh_layers) {
  CHECK_NOTNULL(transformed_sub_map_mesh_layers);
  transformed_sub_map_mesh_layers->clear();
  transformed_sub_map_mesh_layers->reserve(sub_map_mesh_layers.size());
  // Looping over all the mesh layers for the submaps
  LOG(INFO) << "Starting Transforming mesh layers.";
  for (size_t sub_map_index = 0; sub_map_index < sub_map_mesh_layers.size();
       sub_map_index++) {
    // Getting the mesh layer and the transform
    const Transformation& T_M_S = sub_map_poses[sub_map_index];
    MeshLayer::ConstPtr mesh_layer_ptr = sub_map_mesh_layers[sub_map_index];
    // Making a new layer for the transformed triangles
    MeshLayer::Ptr transformed_mesh_layer_ptr =
        transformMeshLayer(*mesh_layer_ptr, T_M_S);
    // Adding to the mesh layer vector
    transformed_sub_map_mesh_layers->push_back(transformed_mesh_layer_ptr);
  }
}

MeshLayer::Ptr SubmapMesher::transformMeshLayer(
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

void SubmapMesher::colorMeshLayersWithIndex(
    std::vector<MeshLayer::Ptr>* sub_map_mesh_layers) {
  // Color map from submap IDs to unique colors
  static const voxblox::ExponentialOffsetIdColorMap submap_id_color_map;

  // Looping over submaps and coloring
  size_t num_sub_maps = sub_map_mesh_layers->size();
  for (size_t sub_map_index = 0; sub_map_index < num_sub_maps;
       sub_map_index++) {
    // Extracting the mesh layer
    MeshLayer* mesh_layer_ptr = ((*sub_map_mesh_layers)[sub_map_index]).get();
    // Generating a color
    Color sub_map_color = submap_id_color_map.colorLookup(sub_map_index);
    // Coloring this mesh layer
    colorMeshLayer(sub_map_color, mesh_layer_ptr);
  }
}

void SubmapMesher::colorMeshLayer(const Color& color_in,
                                  MeshLayer* mesh_layer_ptr) {
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

void SubmapMesher::transformAndAddTrianglesToLayer(
    const MeshLayer& input_mesh_layer, const Transformation& T_B_A,
    MeshLayer* output_mesh_layer) {
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

void SubmapMesher::transformAndAddTriangleToMesh(const Mesh& input_mesh,
                                                 const VertexIndex start_index,
                                                 const Transformation& T_B_A,
                                                 Mesh* output_mesh) {
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
    VertexIndex current_max_index = output_mesh->indices.size();
    output_mesh->indices.push_back(current_max_index + 0);
    output_mesh->indices.push_back(current_max_index + 1);
    output_mesh->indices.push_back(current_max_index + 2);
  }
}

void SubmapMesher::addTrianglesToLayer(const MeshLayer& input_mesh_layer,
                                       MeshLayer* output_mesh_layer) {
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

void SubmapMesher::addTriangleToMesh(const Mesh& input_mesh,
                                     const VertexIndex start_index,
                                     Mesh* output_mesh) {
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
    VertexIndex current_max_index = output_mesh->indices.size();
    output_mesh->indices.push_back(current_max_index + 0);
    output_mesh->indices.push_back(current_max_index + 1);
    output_mesh->indices.push_back(current_max_index + 2);
  }
}

}  // namespace cblox
