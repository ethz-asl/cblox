#include <glog/logging.h>
#include <ros/ros.h>

#include "cblox_ros/tsdf_submap_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cblox");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  cblox::TsdfSubmapServer node(nh, nh_private);

  ros::spin();
  return 0;
}
