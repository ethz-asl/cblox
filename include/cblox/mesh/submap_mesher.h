#ifndef CBLOX_MESH_SUBMAP_MESHER_H_
#define CBLOX_MESH_SUBMAP_MESHER_H_

#include <memory>
#include <vector>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>

#include "cblox/core/common.h"
#include "cblox/core/submap_collection.h"
#include "cblox/core/tsdf_submap.h"

namespace cblox {

using voxblox::Mesh;
using voxblox::MeshLayer;
using voxblox::MeshIntegrator;
using voxblox::MeshIntegratorConfig;
using voxblox::Color;
using voxblox::VertexIndex;

class SubmapMesher {
 public:
  typedef std::shared_ptr<SubmapMesher> Ptr;
  typedef std::shared_ptr<const SubmapMesher> ConstPtr;

  // Constructor
  SubmapMesher(const TsdfMap::Config &submap_config,
               const MeshIntegratorConfig &mesh_config)
      : tsdf_map_config_(submap_config), mesh_config_(mesh_config) {}

  // Generating various meshes
  template <typename SubmapType>
  void generateSeparatedMesh(
      const SubmapCollection<SubmapType> &submap_collection,
      MeshLayer *seperated_mesh_layer_ptr);
  template <typename SubmapType>
  void generateCombinedMesh(
      const SubmapCollection<SubmapType> &submap_collection,
      MeshLayer *combined_mesh_layer_ptr);
  void generatePatchMeshes(
      const SubmapCollection<TsdfSubmap> &tsdf_submap_collection,
      std::vector<MeshLayer::Ptr> *sub_map_mesh_layers);

  // NOTE(alex.millane): Generates a mesh to test interpolation methods.
  //                     Transforms the TSDF then generates meshses and combines
  // TODO(alex.millane): Remove once happy with the interpolator performance.
  void generateInterpolationTestMesh(
      const SubmapCollection<TsdfSubmap> &tsdf_submap_collection,
      MeshLayer *interpolation_test_mesh_layer_ptr);

  // Generates a mesh from submaps which have been trimmed in height
  // TODO(alex.millane): This is kind of filthy, but useful for evaluation.
  void generateTrimmedCombinedMesh(
      const SubmapCollection<TsdfSubmap> &tsdf_submap_collection,
      double mesh_trim_height, MeshLayer *combined_mesh_layer_ptr);

  // Generates mesh layers from the TSDF submaps
  template <typename SubmapType>
  void generateSeparatedMeshLayers(
      const std::vector<typename SubmapType::Ptr> &sub_maps,
      std::vector<MeshLayer::Ptr> *sub_map_mesh_layers);

  // Transforms a vector of mesh layers by a vector of posses
  void transformMeshLayers(
      const std::vector<MeshLayer::Ptr> &sub_map_mesh_layers,
      const AlignedVector<Transformation> &sub_map_poses,
      std::vector<MeshLayer::Ptr> *transformed_sub_map_mesh_layers);
  // Transforms a single mesh layer
  MeshLayer::Ptr transformMeshLayer(const MeshLayer &mesh_layer,
                                    const Transformation &transformation);

  // Combine the mesh layers
  void combineMeshLayers(const std::vector<MeshLayer::Ptr> &sub_map_mesh_layers,
                         const AlignedVector<Transformation> &submap_poses,
                         MeshLayer *combined_mesh_layer_ptr);

  // Transform and add funcions
  // TODO(alex.millane): There's a lot of code duplication in the
  //                     implementation. Clean up.
  void transformAndAddTrianglesToLayer(const MeshLayer &input_mesh_layer,
                                       const Transformation &T_B_A,
                                       MeshLayer *output_mesh_layer);
  void transformAndAddTriangleToMesh(const Mesh &input_mesh,
                                     const VertexIndex start_index,
                                     const Transformation &T_B_A,
                                     Mesh *output_mesh);

  // Add without transforming functions
  // TODO(alex.millane): There's a lot of code duplication in the
  //                     implementation. Clean up.
  void addTrianglesToLayer(const MeshLayer &input_mesh_layer,
                           MeshLayer *output_mesh_layer);
  void addTriangleToMesh(const Mesh &input_mesh, const VertexIndex start_index,
                         Mesh *output_mesh);

  // Functions for coloring meshes
  void colorMeshLayersWithIndex(
      std::vector<MeshLayer::Ptr> *sub_map_mesh_layers) const;
  void colorMeshLayer(const Color &color_in, MeshLayer *mesh_layer_ptr) const;

  // Functions for interacting with the submap collection
  // TODO(alex.millane): Should be encapsulated into the submap collection
  void trimSubmapToHeight(const float trim_height,
                          const TsdfSubmap &tsdf_sub_map,
                          TsdfSubmap *trimmed_tsdf_sub_map_ptr) const;

 private:
  // The configs
  const TsdfMap::Config tsdf_map_config_;
  const MeshIntegratorConfig mesh_config_;
};

}  // namespace cblox

#include "cblox/mesh/submap_mesher_inl.h"

#endif  // CBLOX_MESH_SUBMAP_MESHER_H_
