cmake_minimum_required(VERSION 3.5)
project(src)

set(CMAKE_CXX_STANDARD 14)

add_executable(src
        core/tsdf_esdf_submap.cpp
        core/tsdf_submap.cpp
        integrator/tsdf_submap_collection_integrator.cpp
        io/transformation_io.cpp
        io/tsdf_submap_io.cpp
        mesh/submap_mesher.cpp
        utils/quat_transformation_protobuf_utils.cpp)
