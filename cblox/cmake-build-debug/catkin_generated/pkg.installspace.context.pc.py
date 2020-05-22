# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;eigen_catkin;glog_catkin;voxblox;voxblox_ros;minkindr;cblox_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lcblox_lib".split(';') if "-lcblox_lib" != "" else []
PROJECT_NAME = "cblox"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
