#ifndef CBLOX_ROS_ROS_PARAMS_H_
#define CBLOX_ROS_ROS_PARAMS_H_

#include <ros/node_handle.h>

#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_ros/ros_params.h>

#include "cblox/core/tsdf_esdf_submap.h"
#include "cblox/core/tsdf_submap.h"

namespace cblox {

inline voxblox::TsdfIntegratorType getTsdfIntegratorTypeFromRosParam(
    const ros::NodeHandle& nh_private) {
  voxblox::TsdfIntegratorType tsdf_integrator_type;
  std::string integrator_type_str("merged");
  nh_private.param("method", integrator_type_str, integrator_type_str);
  int integrator_type_idx = 1;
  bool valid_flag = false;
  for (const std::string& valid_integrator_type_name :
       voxblox::kTsdfIntegratorTypeNames) {
    if (integrator_type_str == valid_integrator_type_name) {
      tsdf_integrator_type =
          static_cast<voxblox::TsdfIntegratorType>(integrator_type_idx);
      valid_flag = true;
      break;
    }
    ++integrator_type_idx;
  }
  if (!valid_flag) {
    LOG(FATAL) << "Unknown TSDF integrator type: " << integrator_type_str;
  }

  return tsdf_integrator_type;
}

template <typename SubmapType>
inline typename SubmapType::Config getSubmapConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  auto tsdf_map_config = voxblox::getTsdfMapConfigFromRosParam(nh_private);
  return typename SubmapType::Config(tsdf_map_config);
}

template <>
inline typename TsdfEsdfSubmap::Config
getSubmapConfigFromRosParam<TsdfEsdfSubmap>(const ros::NodeHandle& nh_private) {
  auto tsdf_map_config = voxblox::getTsdfMapConfigFromRosParam(nh_private);
  auto esdf_map_config = voxblox::getEsdfMapConfigFromRosParam(nh_private);
  return TsdfEsdfSubmap::Config(tsdf_map_config, esdf_map_config);
}

}  // namespace cblox

#endif  // CBLOX_ROS_ROS_PARAMS_H_
