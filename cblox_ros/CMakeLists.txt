cmake_minimum_required(VERSION 2.8.3)
project(cblox_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/active_submap_visualizer.cc
  src/trajectory_visualizer.cc
  src/submap_server.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############

cs_add_executable(tsdf_submap_server
  src/tsdf_submap_server_node.cc
)
target_link_libraries(tsdf_submap_server ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
