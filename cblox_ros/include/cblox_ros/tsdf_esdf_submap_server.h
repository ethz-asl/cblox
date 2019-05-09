#ifndef CBLOX_ROS_TSDF_ESDF_SUBMAP_SERVER_H
#define CBLOX_ROS_TSDF_ESDF_SUBMAP_SERVER_H

#include <cblox_ros/tsdf_submap_server.h>

namespace cblox {

class TsdfEsdfSubmapServer : public TsdfSubmapServer {

 public:
  SubmapCollection<TsdfEsdfSubmap>::Ptr getTsdfEsdfSubmapCollectionPtr();

 private:
  void finishSubmap();

  std::shared_ptr <SubmapCollection<TsdfEsdfSubmap>> submap_collection_ptr_;
};

}
#endif //CBLOX_ROS_TSDF_ESDF_SUBMAP_SERVER_H
