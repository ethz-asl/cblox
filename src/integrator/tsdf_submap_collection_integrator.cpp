#include "cblox/integrator/tsdf_submap_collection_integrator.h"

namespace cblox {

TsdfSubmapCollectionIntegrator::TsdfSubmapCollectionIntegrator(
    const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
    const voxblox::TsdfIntegratorType& tsdf_integrator_type,
    const std::shared_ptr<TsdfSubmapCollection>& tsdf_submap_collection_ptr)
    : tsdf_submap_collection_ptr_(tsdf_submap_collection_ptr),
      method_(tsdf_integrator_type) {}

void TsdfSubmapCollectionIntegrator::integratePointCloud(
    const Transformation& T_M_C, const Pointcloud& points_C,
    const Colors& colors) {
  // Passing data to the tsdf integrator
  tsdf_integrator_->integratePointCloud(T_M_C, points_C, colors);
}

void TsdfSubmapCollectionIntegrator::activateLatestSubmap() {
  // Setting the server members to point to this submap
  // NOTE(alexmillane): This is slightly confusing because the collection is
  //                    increased in size elsewhere but we change the
  //                    integration target to the new submap here. The result is
  //                    that between new submap creation and activation the
  //                    integrator wont be affecting the latest submap in the
  //                    collection.
  updateIntegratorTarget(tsdf_submap_collection_ptr_->getActiveTsdfMapPtr());
}

void TsdfSubmapCollectionIntegrator::initializeIntegrator(
    const TsdfMap::Ptr& tsdf_map_ptr) {
  CHECK(tsdf_map_ptr);
  // Creating with the voxblox provided factory
  tsdf_integrator_ = voxblox::TsdfIntegratorFactory::create(
      method_, tsdf_integrator_config_, tsdf_map_ptr->getTsdfLayerPtr());
}

void TsdfSubmapCollectionIntegrator::updateIntegratorTarget(
    const TsdfMap::Ptr& tsdf_map_ptr) {
  CHECK(tsdf_map_ptr);
  // Creating the integrator if not yet created.
  // Otherwise, changing the integration target.
  if (tsdf_integrator_ == nullptr) {
    initializeIntegrator(tsdf_map_ptr);
  } else {
    tsdf_integrator_->setLayer(tsdf_map_ptr->getTsdfLayerPtr());
  }
}

}  // namespace cblox
