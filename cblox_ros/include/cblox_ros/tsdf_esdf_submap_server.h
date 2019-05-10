#ifndef CBLOX_ROS_TSDF_ESDF_SUBMAP_SERVER_H
#define CBLOX_ROS_TSDF_ESDF_SUBMAP_SERVER_H

#include <cblox_ros/tsdf_submap_server.h>

namespace cblox {

class TsdfEsdfSubmapServer : public TsdfSubmapServer {
 public:
  // Constructor
  TsdfEsdfSubmapServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private);
  TsdfEsdfSubmapServer(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      const TsdfMap::Config& tsdf_map_config,
      const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
      const voxblox::TsdfIntegratorType& tsdf_integrator_type,
      const voxblox::MeshIntegratorConfig& mesh_config);
  virtual ~TsdfEsdfSubmapServer() {}

  SubmapCollection<TsdfEsdfSubmap>::Ptr getTsdfEsdfSubmapCollectionPtr() const;

 private:
  // void createNewSubMap(const Transformation& T_G_C);
  void finishSubmap();

  std::shared_ptr <SubmapCollection<TsdfEsdfSubmap>> submap_collection_ptr_;
};

}
#endif //CBLOX_ROS_TSDF_ESDF_SUBMAP_SERVER_H
