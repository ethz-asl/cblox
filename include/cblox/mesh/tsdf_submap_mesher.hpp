#ifndef CBLOX_MESH_TSDF_SUBMAP_MESHER_H_
#define CBLOX_MESH_TSDF_SUBMAP_MESHER_H_

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>

#include "cblox/core/common.hpp"
#include "cblox/core/tsdf_submap.hpp"
#include "cblox/core/tsdf_submap_collection.hpp"

namespace cblox {

using namespace voxblox;

class TsdfSubmapMesher {
 public:
  typedef std::shared_ptr<TsdfSubmapMesher> Ptr;
  typedef std::shared_ptr<const TsdfSubmapMesher> ConstPtr;

  // Constructor
  TsdfSubmapMesher(const TsdfMap::Config& tsdf_map_config,
                   const MeshIntegratorConfig& mesh_config)
      : tsdf_map_config_(tsdf_map_config), mesh_config_(mesh_config){};

  // Generating various meshes
  void generateSeparatedMesh(const TsdfSubmapCollection &tsdf_submap_collection,
                             MeshLayer *seperated_mesh_layer_ptr);
  void generateCombinedMesh(const TsdfSubmapCollection &tsdf_submap_collection,
                            MeshLayer *combined_mesh_layer_ptr);
  void generatePatchMeshes(const TsdfSubmapCollection &tsdf_submap_collection,
                           std::vector<MeshLayer::Ptr> *sub_map_mesh_layers);

  // NOTE(alex.millane): Generates a mesh to test interpolation methods.
  //                     Transforms the TSDF then generates meshses and combines
  // TODO(alex.millane): Remove once happy with the interpolator performance.
  void generateInterpolationTestMesh(
      const TsdfSubmapCollection &tsdf_submap_collection,
      MeshLayer *interpolation_test_mesh_layer_ptr);

  // Generates a mesh from submaps which have been trimmed in height
  // TODO(alex.millane): This is kind of filthy, but useful for evaluation.
  void generateTrimmedCombinedMesh(
      const TsdfSubmapCollection &tsdf_submap_collection,
      double mesh_trim_height, MeshLayer *combined_mesh_layer_ptr);

  // Generates mesh layers from the TSDF submaps
  void generateSeparatedMeshLayers(
      const std::vector<TsdfSubmap::Ptr>& tsdf_sub_maps,
      std::vector<MeshLayer::Ptr>* sub_map_mesh_layers);

  // Transforms a vector of mesh layers by a vector of posses
  void transformMeshLayers(
      const std::vector<MeshLayer::Ptr>& sub_map_mesh_layers,
      const AlignedVector<Transformation>& sub_map_poses,
      std::vector<MeshLayer::Ptr>* transformed_sub_map_mesh_layers);
  // Transforms a single mesh layer
  MeshLayer::Ptr transformMeshLayer(const MeshLayer& mesh_layer,
                                    const Transformation& transformation);

  // Combine the mesh layers
  void combineMeshLayers(const std::vector<MeshLayer::Ptr>& sub_map_mesh_layers,
                         const AlignedVector<Transformation>& submap_poses,
                         MeshLayer* combined_mesh_layer_ptr);

  // Transform and add funcions
  // TODO(alex.millane): There's a lot of code duplication in the
  //                     implementation. Clean up.
  void transformAndAddTrianglesToLayer(const MeshLayer& input_mesh_layer,
                                       const Transformation& T_B_A,
                                       MeshLayer* output_mesh_layer);
  void transformAndAddTriangleToMesh(const Mesh& input_mesh,
                                     const VertexIndex start_index,
                                     const Transformation& T_B_A,
                                     Mesh* output_mesh);

  // Add without transforming functions
  // TODO(alex.millane): There's a lot of code duplication in the
  //                     implementation. Clean up.
  void addTrianglesToLayer(const MeshLayer& input_mesh_layer,
                           MeshLayer* output_mesh_layer);
  void addTriangleToMesh(const Mesh& input_mesh, const VertexIndex start_index,
                         Mesh* output_mesh);

  // Functions for coloring meshes
  void colorMeshLayersWithIndex(
      std::vector<MeshLayer::Ptr>* sub_map_mesh_layers) const;
  void colorMeshLayer(const Color& color_in, MeshLayer* mesh_layer_ptr) const;

  // Functions for interacting with the submap collection
  // TODO(alex.millane): Should be encapsulated into the submap collection
  void trimSubmapToHeight(const float trim_height,
                          const TsdfSubmap& tsdf_sub_map,
                          TsdfSubmap* trimmed_tsdf_sub_map_ptr) const;

 private:
  // The configs
  const TsdfMap::Config tsdf_map_config_;
  const MeshIntegratorConfig mesh_config_;
};

}  // namespace cblox

#endif /* CBLOX_MESH_TSDF_SUBMAP_MESHER_H_ */
