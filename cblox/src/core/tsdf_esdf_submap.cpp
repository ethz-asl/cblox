#include "cblox/core/tsdf_esdf_submap.h"
#include "cblox/utils/quat_transformation_protobuf_utils.h"

namespace cblox {

void TsdfEsdfSubmap::generateEsdf() {
  // Instantiate the integrator.
  voxblox::EsdfIntegrator esdf_integrator(esdf_integrator_config_,
                                          tsdf_map_->getTsdfLayerPtr(),
                                          esdf_map_->getEsdfLayerPtr());
  // Generate the ESDF.
  LOG(INFO) << "Generating ESDF from TSDF for submap with ID: " << submap_id_;
  esdf_integrator.updateFromTsdfLayerBatch();
}

void TsdfEsdfSubmap::finishSubmap() { generateEsdf(); }

void TsdfEsdfSubmap::prepareForPublish() { generateEsdf(); }

void TsdfEsdfSubmap::getProto(cblox::SubmapProto* proto) const {
  CHECK_NOTNULL(proto);
  TsdfSubmap::getProto(proto);

  // Add ESDF info.
  size_t num_blocks = esdf_map_->getEsdfLayer().getNumberOfAllocatedBlocks();
  proto->set_num_esdf_blocks(num_blocks);
}

bool TsdfEsdfSubmap::saveToStream(std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  bool success = TsdfSubmap::saveToStream(outfile_ptr);
  if (!success) {
    return false;
  }

  // Saving ESDF layer.
  constexpr bool kIncludeAllBlocks = true;
  const Layer<EsdfVoxel>& esdf_layer = esdf_map_->getEsdfLayer();
  if (!esdf_layer.saveBlocksToStream(kIncludeAllBlocks,
                                     voxblox::BlockIndexList(), outfile_ptr)) {
    LOG(ERROR) << "Could not write sub map blocks to stream.";
    outfile_ptr->close();
    return false;
  }

  // Success.
  return true;
}

TsdfEsdfSubmap::Ptr TsdfEsdfSubmap::LoadFromStream(
    const Config& config, std::istream* proto_file_ptr,
    uint64_t* tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);

  // Getting the header for this submap.
  SubmapProto submap_proto;
  if (!voxblox::utils::readProtoMsgFromStream(proto_file_ptr, &submap_proto,
                                              tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not read tsdf sub map protobuf message.";
    return nullptr;
  }

  // Getting the transformation.
  Transformation T_M_S;
  QuatTransformationProto transformation_proto = submap_proto.transform();
  conversions::transformProtoToKindr(transformation_proto, &T_M_S);

  LOG(INFO) << "Submap id: " << submap_proto.id();
  Eigen::Vector3 t = T_M_S.getPosition();
  Quaternion q = T_M_S.getRotation();
  LOG(INFO) << "[ " << t.x() << ", " << t.y() << ", " << t.z() << ", " << q.w()
            << ", " << q.x() << ", " << q.y() << ", " << q.z() << " ]";

  // Creating a new submap to hold the data.
  auto submap_ptr =
      std::make_shared<TsdfEsdfSubmap>(T_M_S, submap_proto.id(), config);

  // Getting the tsdf blocks for this submap (the tsdf layer).
  LOG(INFO) << "Tsdf number of allocated blocks: " << submap_proto.num_blocks();
  if (!voxblox::io::LoadBlocksFromStream(
          submap_proto.num_blocks(),
          Layer<TsdfVoxel>::BlockMergingStrategy::kReplace, proto_file_ptr,
          submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr(),
          tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not load the tsdf blocks from stream.";
    return nullptr;
  }
  // Getting the esdf blocks for this submap (the esdf layer).
  LOG(INFO) << "Esdf number of allocated blocks: "
            << submap_proto.num_esdf_blocks();
  if (submap_proto.num_esdf_blocks() == 0) {
    LOG(WARNING)
        << "Number of ESDF blocks is zero. You may be loading TSDFSubmap.";
  }
  if (!voxblox::io::LoadBlocksFromStream(
          submap_proto.num_esdf_blocks(),
          Layer<EsdfVoxel>::BlockMergingStrategy::kReplace, proto_file_ptr,
          submap_ptr->getEsdfMapPtr()->getEsdfLayerPtr(),
          tmp_byte_offset_ptr)) {
    LOG(ERROR) << "Could not load the esdf blocks from stream.";
    return nullptr;
  }

  return submap_ptr;
}

}  // namespace cblox
