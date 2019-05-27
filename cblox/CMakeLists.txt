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
# Taken from voxblox
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
set(PROTO_DEFNS proto/cblox/QuatTransformation.proto
                proto/cblox/TsdfSubmap.proto
                proto/cblox/TsdfSubmapCollection.proto)
set(ADDITIONAL_LIBRARIES "")

find_package(protobuf_catkin QUIET)
if (protobuf_catkin_FOUND)
    message(STATUS "Using protobuf_catkin")
    list(APPEND catkin_INCLUDE_DIRS ${protobuf_catkin_INCLUDE_DIRS})
    list(APPEND catkin_LIBRARIES ${protobuf_catkin_LIBRARIES})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
    set(ADDITIONAL_LIBRARIES ${protobuf_catkin_LIBRARIES})
else()
  message(STATUS "Using system protobuf")
  find_package(Protobuf REQUIRED)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
  set(ADDITIONAL_LIBRARIES ${PROTOBUF_LIBRARY})
endif()

#############
# LIBRARIES #
#############
cs_add_library(cblox_lib
  src/core/tsdf_submap.cpp
  src/core/tsdf_esdf_submap.cpp
  src/core/planning_submap.cpp
  src/integrator/tsdf_submap_collection_integrator.cpp
  src/utils/quat_transformation_protobuf_utils.cpp
  src/mesh/submap_mesher.cpp
  src/io/tsdf_submap_io.cpp
  src/io/transformation_io.cpp
  ${PROTO_SRCS}
)
target_link_libraries(cblox_lib ${catkin_LIBRARIES})

##########
# EXPORT #
##########

cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          CFG_EXTRAS cblox-extras.cmake)