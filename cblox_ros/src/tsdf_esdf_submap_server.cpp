#include "cblox_ros/tsdf_esdf_submap_server.h"
#include "../include/cblox_ros/tsdf_submap_server.h"

namespace cblox {

SubmapCollection<TsdfEsdfSubmap>::Ptr
    TsdfEsdfSubmapServer::getTsdfEsdfSubmapCollectionPtr() {
  return submap_collection_ptr_;
}

void TsdfEsdfSubmapServer::finishSubmap() {
  TsdfSubmapServer::finishSubmap();
  if (submap_collection_ptr_->getActiveSubMapPtr()) {
    submap_collection_ptr_->getActiveSubMapPtr()->generateEsdf();
  }
}

}