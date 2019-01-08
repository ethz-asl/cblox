//
// Created by victor on 07.01.19.
//

#include "cblox/core/tsdf_esdf_submap.h"
#include <voxblox/integrator/esdf_integrator.h>

namespace cblox {
void TsdfEsdfSubmap::generateEsdf() {
  // Instantiate the integrator
  voxblox::EsdfIntegrator::Config esdf_integrator_config;
  voxblox::EsdfIntegrator esdf_integrator(esdf_integrator_config,
                                          tsdf_map_->getTsdfLayerPtr(),
                                          esdf_map_->getEsdfLayerPtr());
  // Generate the ESDF
  std::cout << "Generating ESDF from TSDF for submap with ID: " << submap_id_
            << std::endl;
  esdf_integrator.updateFromTsdfLayerBatch();
}
}  // namespace cblox
