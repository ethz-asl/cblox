#include "cblox_ros/tsdf_esdf_submap_server.h"
#include <cblox_ros/tsdf_submap_server.h>
#include "cblox_ros/ros_params.h"
#include <voxblox_ros/ros_params.h>

namespace cblox {
TsdfEsdfSubmapServer::TsdfEsdfSubmapServer(const ros::NodeHandle& nh,
                                           const ros::NodeHandle& nh_private)
    : TsdfEsdfSubmapServer(
    nh, nh_private, voxblox::getTsdfMapConfigFromRosParam(nh_private),
    voxblox::getTsdfIntegratorConfigFromRosParam(nh_private),
    getTsdfIntegratorTypeFromRosParam(nh_private),
    voxblox::getMeshIntegratorConfigFromRosParam(nh_private)) {}

TsdfEsdfSubmapServer::TsdfEsdfSubmapServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const TsdfMap::Config& tsdf_map_config,
    const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
    const voxblox::TsdfIntegratorType& tsdf_integrator_type,
    const voxblox::MeshIntegratorConfig& mesh_config)
    : TsdfSubmapServer(nh, nh_private, tsdf_map_config, tsdf_integrator_config,
                      tsdf_integrator_type, mesh_config) {}

SubmapCollection<TsdfEsdfSubmap>::Ptr
    TsdfEsdfSubmapServer::getTsdfEsdfSubmapCollectionPtr() const {
  return submap_collection_ptr_;
}

void TsdfEsdfSubmapServer::finishSubmap() {
  TsdfSubmapServer::finishSubmap();
  if (tsdf_submap_collection_ptr_->exists(
      tsdf_submap_collection_ptr_->getActiveSubMapID())) {
    submap_collection_ptr_->getActiveSubMapPtr()->generateEsdf();
  }
}

}