#include "cblox/core/tsdf_esdf_submap.h"

namespace cblox {

void TsdfEsdfSubmap::generateEsdf() {
  // Instantiate the integrator
  voxblox::EsdfIntegrator esdf_integrator(esdf_integrator_config_,
                                          tsdf_map_->getTsdfLayerPtr(),
                                          esdf_map_->getEsdfLayerPtr());
  // Generate the ESDF
  LOG(INFO) << "Generating ESDF from TSDF for submap with ID: " << submap_id_;
  esdf_integrator.setFullEuclidean(full_euclidian);
  esdf_integrator.updateFromTsdfLayerBatch();
}

void TsdfEsdfSubmap::setTsdfMap(const voxblox::Layer<TsdfVoxel>& tsdf_layer) {
  tsdf_map_.reset(new voxblox::TsdfMap(tsdf_layer));
}
}  // namespace cblox
