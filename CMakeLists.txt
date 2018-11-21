cmake_minimum_required(VERSION 2.8.0)
project(cblox)

add_definitions(-std=c++11 -march=native)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Mark these as system deps to supress warnings.
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

############
# PROTOBUF #
############
set(PROTO_DEFNS proto/cblox/QuatTransformation.proto
                proto/cblox/TsdfSubmap.proto
                proto/cblox/TsdfSubmapCollection.proto)
PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#############
# LIBRARIES #
#############
cs_add_library(cblox_lib
  src/core/tsdf_submap_collection.cpp
  src/core/tsdf_submap.cpp
  src/utils/quat_transformation_protobuf_utils.cpp
  src/mesh/tsdf_submap_mesher.cpp
  src/io/tsdf_submap_io.cpp
  src/io/transformation_io.cpp
  ${PROTO_SRCS}
)
target_link_libraries(cblox_lib ${catkin_LIBRARIES})

############
# BINARIES #
############
#cs_add_executable(cblox_node
#  src/cblox_node.cpp
#)
#target_link_libraries(cblox_node cblox_lib ${catkin_LIBRARIES})

##########
# EXPORT #
##########

cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          CFG_EXTRAS cblox-extras.cmake)