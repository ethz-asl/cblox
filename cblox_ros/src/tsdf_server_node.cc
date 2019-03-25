#include "cblox_ros/tsdf_server.h"

#include <glog/logging.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cblox");
  google::InitGoogleLogging(argv[0]);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  cblox::TsdfServer node(nh, nh_private);

  ros::spin();
  return 0;
}
