#include "cblox/core/tsdf_esdf_submap.h"
#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {

void TsdfEsdfSubmap::generateEsdf() {
  // Instantiate the integrator
  voxblox::EsdfIntegrator esdf_integrator(esdf_integrator_config_,
                                          tsdf_map_->getTsdfLayerPtr(),
                                          esdf_map_->getEsdfLayerPtr());
  // Generate the ESDF
  LOG(INFO) << "Generating ESDF from TSDF for submap with ID: " << submap_id_;
  esdf_integrator.setFullEuclidean(false);

  std::unique_lock<std::mutex> lock(esdf_mutex);
  esdf_integrator.updateFromTsdfLayerBatch();
}

void TsdfEsdfSubmap::setTsdfMap(const voxblox::Layer<TsdfVoxel>& tsdf_layer) {
  tsdf_map_.reset(new voxblox::TsdfMap(tsdf_layer));
}

void TsdfEsdfSubmap::getProto(cblox::SubmapProto *proto) const {
  CHECK_NOTNULL(proto);
  TsdfSubmap::getProto(proto);

  // add ESDF info
  size_t num_blocks = esdf_map_->getEsdfLayer().getNumberOfAllocatedBlocks();
  proto->set_num_esdf_blocks(num_blocks);
}
bool TsdfEsdfSubmap::saveToStream(std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  bool success = TsdfSubmap::saveToStream(outfile_ptr);
  if (!success) {
    return false;
  }

  // Saving ESDF layer
  constexpr bool kIncludeAllBlocks = true;
  const Layer<voxblox::EsdfVoxel>& esdf_layer = esdf_map_->getEsdfLayer();
  if (!esdf_layer.saveBlocksToStream(kIncludeAllBlocks,
                                     voxblox::BlockIndexList(), outfile_ptr)) {
    LOG(ERROR) << "Could not write sub map blocks to stream.";
    outfile_ptr->close();
    return false;
  }

  // Success
  return true;
}

void TsdfEsdfSubmap::serializeToMsg(cblox_msgs::MapLayer* msg) const {
  // set type to ESDF
  msg->type = 0;

  TsdfSubmap::serializeToMsg(msg);

  voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(
      esdf_map_->getEsdfLayer(), false, &msg->esdf_layer);
  msg->esdf_layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
}
bool TsdfEsdfSubmap::deserializeFromMsg(cblox_msgs::MapLayer* msg) {

  // read tsdf layer
  bool tsdf_success = TsdfSubmap::deserializeFromMsg(msg);
  // TODO: esdf dependent on tsdf
  //       change at the same time, even if info not available?
  bool esdf_success = voxblox::deserializeMsgToLayer(msg->esdf_layer,
      esdf_map_->getEsdfLayerPtr());

  // generate ESDF layer if necessary
  if (tsdf_success and !esdf_success) {
    generateEsdf();
  }

  return tsdf_success && esdf_success;
}

}  // namespace cblox
