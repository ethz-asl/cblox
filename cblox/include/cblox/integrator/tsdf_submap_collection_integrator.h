#ifndef CBLOX_INTEGRATOR_TSDF_SUBMAP_COLLECTION_INTEGRATOR_H_
#define CBLOX_INTEGRATOR_TSDF_SUBMAP_COLLECTION_INTEGRATOR_H_

#include <memory>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>

#include "cblox/core/common.h"
#include "cblox/core/submap_collection.h"
#include "cblox/core/tsdf_submap.h"

namespace cblox {

class TsdfSubmapCollectionIntegrator {
 public:
  TsdfSubmapCollectionIntegrator(
      const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
      const voxblox::TsdfIntegratorType& tsdf_integrator_type,
      const std::shared_ptr<SubmapCollectionInterface>&
          tsdf_submap_collection_ptr)
      : tsdf_submap_collection_ptr_(tsdf_submap_collection_ptr),
        tsdf_integrator_config_(tsdf_integrator_config),
        method_(tsdf_integrator_type) {}

  // Integrate a pointcloud to the currently active submap.
  // NOTE(alexmilane): T_G_S - Transformation between camera frame (C) and
  //                           the global tracking frame (G).
  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors);

  // Changes the active submap to the last one on the collection
  void switchToActiveSubmap();

 private:
  // Initializes the integrator
  void initializeIntegrator(const TsdfMap::Ptr& tsdf_map_ptr);

  // Changes the integration target the latest submap in the collection.
  void updateIntegratorTarget(const TsdfMap::Ptr& tsdf_map_ptr);

  // Gets the submap relative pose
  // NOTE(alexmilane): T_G_S - Transformation between camera frame (C) and
  //                           the global tracking frame (G).
  Transformation getSubmapRelativePose(const Transformation& T_G_C) const;

  // The submap collection
  std::shared_ptr<SubmapCollectionInterface> tsdf_submap_collection_ptr_;

  // Transform to the currently targeted submap
  // NOTE(alexmilane): T_G_S - Transformation between Submap base frame (S) and
  //                           the global tracking frame (G).
  Transformation T_G_S_active_;

  // The integrator
  const voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  voxblox::TsdfIntegratorBase::Ptr tsdf_integrator_;

  // Merging method for integrating new pointclouds
  const voxblox::TsdfIntegratorType method_;
};

}  // namespace cblox

#endif  // CBLOX_INTEGRATOR_TSDF_SUBMAP_COLLECTION_INTEGRATOR_H_
