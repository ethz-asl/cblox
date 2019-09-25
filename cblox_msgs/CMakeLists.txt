cmake_minimum_required(VERSION 2.8.3)
project(cblox_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    voxblox_msgs
    message_generation
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
    FILES
    MapHeader.msg
    MapLayer.msg
    MapPoseEstimate.msg
    MapPoseUpdate.msg
)

add_service_files(
    FILES
    SubmapSrv.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    voxblox_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

##########
# EXPORT #
##########

#cs_install()
#cs_export()
