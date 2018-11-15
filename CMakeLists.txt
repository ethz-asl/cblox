cmake_minimum_required(VERSION 2.8.0)
project(cblox)

add_definitions(-std=c++11 -march=native)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Mark these as system deps to supress warnings.
#include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

############
# PROTOBUF #
############
#[[set(PROTO_DEFNS proto/manifold_mapping/QuatTransformation.proto
                proto/manifold_mapping/TsdfSubMap.proto
                proto/manifold_mapping/TsdfManifoldMap.proto)
PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
]]
#############
# LIBRARIES #
#############
cs_add_library(cblox_lib
  #src/library/core/manifold_mapper.cpp
  src/core/tsdf_submap_collection.cpp
  src/core/tsdf_submap.cpp
  #src/library/core/frame_integrator.cpp
  #src/library/core/transformer.cpp
  #src/library/core/manifold_maintainer.cpp
  #src/library/utils/quat_transformation_protobuf_utils.cpp
  #src/library/utils/conversions.cpp
  #src/library/mesh/manifold_mesher.cpp
  #      src/library/io/tsdf_submap_io.cpp
  #src/library/io/matrix_io.cpp
  #${PROTO_SRCS}
)
target_link_libraries(cblox_lib ${catkin_LIBRARIES})

############
# BINARIES #
############
#cs_add_executable(manifold_mapping_node
#  src/manifold_mapping_node.cpp
#)
#target_link_libraries(manifold_mapping_node manifold_mapper ${catkin_LIBRARIES})

##########
# EXPORT #
##########

cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR})
          #CFG_EXTRAS manifold_mapping-extras.cmake)