#include "cblox/integrator/tsdf_submap_collection_integrator.h"

namespace cblox {

void TsdfSubmapCollectionIntegrator::integratePointCloud(
    const Transformation& T_M_C, const Pointcloud& points_C,
    const Colors& colors) {
  // Getting the submap relative transform
  // NOTE(alexmilane): T_S_C - Transformation between Camera frame (C) and
  //                           the submap base frame (S).
  const Transformation T_S_C = getSubmapRelativePose(T_M_C);
  // Passing data to the tsdf integrator
  tsdf_integrator_->integratePointCloud(T_S_C, points_C, colors);
}

void TsdfSubmapCollectionIntegrator::activateLatestSubmap() {
  // Setting the server members to point to this submap
  // NOTE(alexmillane): This is slightly confusing because the collection is
  //                    increased in size elsewhere but we change the
  //                    integration target to the new submap here. The result is
  //                    that between new submap creation and activation the
  //                    integrator wont be affecting the latest submap in the
  //                    collection.
  // TODO(alexmillane): A callback in the Submap collection would make the most sense here.
  updateIntegratorTarget(tsdf_submap_collection_ptr_->getActiveTsdfMapPtr());
  T_M_S_active_ = tsdf_submap_collection_ptr_->getActiveSubMapPose();
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

Transformation TsdfSubmapCollectionIntegrator::getSubmapRelativePose(
    const Transformation& T_M_C) const {
  return (T_M_S_active_.inverse() * T_M_C);
}

}  // namespace cblox
