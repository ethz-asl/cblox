#ifndef CBLOX_INTEGRATOR_TSDF_SUBMAP_COLLECTION_INTEGRATOR_H_
#define CBLOX_INTEGRATOR_TSDF_SUBMAP_COLLECTION_INTEGRATOR_H_

#include <memory.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>

#include "cblox/core/common.h"
#include "cblox/core/tsdf_submap_collection.h"

namespace cblox {

class TsdfSubmapCollectionIntegrator {
 public:
  TsdfSubmapCollectionIntegrator(
      const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
      const voxblox::TsdfIntegratorType& tsdf_integrator_type,
      const std::shared_ptr<cblox::TsdfSubmapCollection>&
          tsdf_submap_collection_ptr);

  // Integrate a pointcloud to the currently active submap.
  void integratePointCloud(const Transformation& T_M_C,
                           const Pointcloud& points_C, const Colors& colors);

  // Changes the active submap to the last one on the collection
  void activateLatestSubmap();

 private:
  // Initializes the integrator
  void initializeIntegrator(const voxblox::TsdfMap::Ptr& tsdf_map_ptr);

  // Changes the integration target the latest submap in the collection.
  void updateIntegratorTarget(const voxblox::TsdfMap::Ptr& tsdf_map_ptr);

  // The submap collection
  std::shared_ptr<cblox::TsdfSubmapCollection> tsdf_submap_collection_ptr_;

  // The integrator
  const voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  voxblox::TsdfIntegratorBase::Ptr tsdf_integrator_;

  // Merging method for integrating new pointclouds
  const voxblox::TsdfIntegratorType method_;
};

}  // namespace cblox

#endif /* CBLOX_INTEGRATOR_TSDF_SUBMAP_COLLECTION_INTEGRATOR_H_ */
