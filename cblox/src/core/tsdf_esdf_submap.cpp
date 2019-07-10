#include "cblox/core/tsdf_esdf_submap.h"

namespace cblox {

void TsdfEsdfSubmap::generateEsdf() {
  // Instantiate the integrator
  esdf_integrator_ = std::make_shared<voxblox::EsdfIntegrator>(esdf_integrator_config_,
                                          tsdf_map_->getTsdfLayerPtr(),
                                          esdf_map_->getEsdfLayerPtr());
  // Generate the ESDF
  LOG(INFO) << "Generating ESDF from TSDF for submap with ID: " << submap_id_;
  esdf_integrator_->updateFromTsdfLayerBatch();
  std::cout << "end of function reached" << std::endl;
  std::cout << "max distance: " << esdf_integrator_->getEsdfMaxDistance() << std::endl;

}
}  // namespace cblox
