#include "cblox_ros/tsdf_submap_server.h"

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <minkindr_conversions/kindr_msg.h>

#include <voxblox/utils/timing.h>
#include <voxblox_ros/ros_params.h>

#include "cblox_ros/pointcloud_conversions.h"
#include "cblox_ros/pose_vis.h"
#include "cblox_ros/ros_params.h"

namespace cblox {

TsdfSubmapServer::TsdfSubmapServer(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : TsdfSubmapServer(
          nh, nh_private, voxblox::getTsdfMapConfigFromRosParam(nh_private),
          voxblox::getTsdfIntegratorConfigFromRosParam(nh_private),
          getTsdfIntegratorTypeFromRosParam(nh_private),
          voxblox::getMeshIntegratorConfigFromRosParam(nh_private)) {}

TsdfSubmapServer::TsdfSubmapServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const TsdfMap::Config& tsdf_map_config,
    const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
    const voxblox::TsdfIntegratorType& tsdf_integrator_type,
    const voxblox::MeshIntegratorConfig& mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      num_integrated_frames_current_submap_(0),
      num_integrated_frames_per_submap_(kDefaultNumFramesPerSubmap),
      color_map_(new voxblox::GrayscaleColorMap()),
      transformer_(nh, nh_private) {
  ROS_DEBUG("Creating a TSDF Server");

  // Initial interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();

  // Creating the submap collection
  tsdf_submap_collection_ptr_.reset(
      new SubmapCollection<TsdfSubmap>(tsdf_map_config));

  // Creating an integrator and targetting the collection
  tsdf_submap_collection_integrator_ptr_.reset(
      new TsdfSubmapCollectionIntegrator(tsdf_integrator_config,
                                         tsdf_integrator_type,
                                         tsdf_submap_collection_ptr_));

  // An object to visualize the submaps
  submap_mesher_ptr_.reset(new SubmapMesher(tsdf_map_config, mesh_config));
  active_submap_visualizer_ptr_.reset(
      new ActiveSubmapVisualizer(mesh_config, tsdf_submap_collection_ptr_));

  // An object to visualize the trajectory
  trajectory_visualizer_ptr_.reset(new TrajectoryVisualizer);
}

void TsdfSubmapServer::subscribeToTopics() {
  // Subscribing to the input pointcloud
  int pointcloud_queue_size = kDefaultPointcloudQueueSize;
  nh_private_.param("pointcloud_queue_size", pointcloud_queue_size,
                    pointcloud_queue_size);
  pointcloud_sub_ = nh_.subscribe("pointcloud", pointcloud_queue_size,
                                  &TsdfSubmapServer::pointcloudCallback, this);
}

void TsdfSubmapServer::advertiseTopics() {
  // Services for saving meshes to file
  generate_separated_mesh_srv_ = nh_private_.advertiseService(
      "generate_separated_mesh",
      &TsdfSubmapServer::generateSeparatedMeshCallback, this);
  generate_combined_mesh_srv_ = nh_private_.advertiseService(
      "generate_combined_mesh", &TsdfSubmapServer::generateCombinedMeshCallback,
      this);
  // Real-time publishing for rviz
  active_submap_mesh_pub_ =
      nh_private_.advertise<visualization_msgs::Marker>("separated_mesh", 1);
  submap_poses_pub_ =
      nh_private_.advertise<geometry_msgs::PoseArray>("submap_baseframes", 1);
  trajectory_pub_ = nh_private_.advertise<nav_msgs::Path>("trajectory", 1);
}

void TsdfSubmapServer::getParametersFromRos() {
  ROS_DEBUG("Getting params from ROS");
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("world_frame", world_frame_, world_frame_);
  // Throttle frame integration
  double min_time_between_msgs_sec = 0.0;
  nh_private_.param("min_time_between_msgs_sec", min_time_between_msgs_sec,
                    min_time_between_msgs_sec);
  min_time_between_msgs_.fromSec(min_time_between_msgs_sec);
  nh_private_.param("mesh_filename", mesh_filename_, mesh_filename_);
  // Timed updates for submap mesh publishing.
  double update_mesh_every_n_sec = 0.0;
  nh_private_.param("update_mesh_every_n_sec", update_mesh_every_n_sec,
                    update_mesh_every_n_sec);
  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
                                &TsdfSubmapServer::updateMeshEvent, this);
  }
  // Frequency of submap creation
  nh_private_.param("num_integrated_frames_per_submap",
                    num_integrated_frames_per_submap_,
                    num_integrated_frames_per_submap_);
}

void TsdfSubmapServer::pointcloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  addMesageToPointcloudQueue(pointcloud_msg_in);
  // Processing messages in the queue
  servicePointcloudQueue();
}

void TsdfSubmapServer::addMesageToPointcloudQueue(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  if (pointcloud_msg_in->header.stamp - last_msg_time_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_ptcloud_ = pointcloud_msg_in->header.stamp;
    pointcloud_queue_.push(pointcloud_msg_in);
  }
}

void TsdfSubmapServer::servicePointcloudQueue() {
  // NOTE(alexmilane): T_G_C - Transformation between Camera frame (C) and
  //                           global tracking frame (G).
  Transformation T_G_C;
  sensor_msgs::PointCloud2::Ptr pointcloud_msg;
  bool processed_any = false;
  while (
      getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = false;

    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);

    if (newSubmapRequired()) {
      createNewSubMap(T_G_C);
    }

    trajectory_visualizer_ptr_->addPose(T_G_C);
    visualizeTrajectory();

    processed_any = true;
  }

  // Note(alex.millane): Currently the timings aren't printing. Outputs too much
  // to the console. But it is occassionally useful so I'm leaving this here.
  constexpr bool kPrintTimings = false;
  if (kPrintTimings) {
    if (processed_any) {
      ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
    }
  }
}

bool TsdfSubmapServer::getNextPointcloudFromQueue(
    std::queue<sensor_msgs::PointCloud2::Ptr>* queue,
    sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C) {
  const size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();
  if (transformer_.lookupTransform((*pointcloud_msg)->header.frame_id,
                                   world_frame_,
                                   (*pointcloud_msg)->header.stamp, T_G_C)) {
    queue->pop();
    return true;
  } else {
    if (queue->size() >= kMaxQueueSize) {
      ROS_ERROR_THROTTLE(60,
                         "Input pointcloud queue getting too long! Dropping "
                         "some pointclouds. Either unable to look up transform "
                         "timestamps or the processing is taking too long.");
      while (queue->size() >= kMaxQueueSize) {
        queue->pop();
      }
    }
  }
  return false;
}

void TsdfSubmapServer::processPointCloudMessageAndInsert(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  // Convert the PCL pointcloud into our awesome format.
  Pointcloud points_C;
  Colors colors;
  convertPointcloudMsg(*color_map_, pointcloud_msg, &points_C, &colors);

  if (verbose_) {
    ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
  }

  if (!mapIntialized()) {
    ROS_INFO("Intializing map.");
    intializeMap(T_G_C);
  }

  ros::WallTime start = ros::WallTime::now();
  integratePointcloud(T_G_C, points_C, colors, is_freespace_pointcloud);
  ros::WallTime end = ros::WallTime::now();
  num_integrated_frames_current_submap_++;
  if (verbose_) {
    ROS_INFO(
        "Finished integrating in %f seconds, have %lu blocks. %u frames "
        "integrated to current submap.",
        (end - start).toSec(), tsdf_submap_collection_ptr_->getActiveTsdfMap()
                                   .getTsdfLayer()
                                   .getNumberOfAllocatedBlocks(),
        num_integrated_frames_current_submap_);
  }
}

void TsdfSubmapServer::integratePointcloud(const Transformation& T_G_C,
                                           const Pointcloud& ptcloud_C,
                                           const Colors& colors,
                                           const bool is_freespace_pointcloud) {
  // Note(alexmillane): Freespace pointcloud option left out for now.
  CHECK_EQ(ptcloud_C.size(), colors.size());
  tsdf_submap_collection_integrator_ptr_->integratePointCloud(T_G_C, ptcloud_C,
                                                              colors);
}

void TsdfSubmapServer::intializeMap(const Transformation& T_G_C) {
  // Just creates the first submap
  createNewSubMap(T_G_C);
}

bool TsdfSubmapServer::newSubmapRequired() const {
  return (num_integrated_frames_current_submap_ >
          num_integrated_frames_per_submap_);
}

void TsdfSubmapServer::createNewSubMap(const Transformation& T_G_C) {
  // Creating the submap
  const SubmapID submap_id =
      tsdf_submap_collection_ptr_->createNewSubMap(T_G_C);
  // Activating the submap in the frame integrator
  tsdf_submap_collection_integrator_ptr_->activateLatestSubmap();
  // Resetting current submap counters
  num_integrated_frames_current_submap_ = 0;

  // Updating the active submap mesher
  active_submap_visualizer_ptr_->activateLatestSubmap();

  // Publish the baseframes
  visualizeSubMapBaseframes();

  if (verbose_) {
    ROS_INFO_STREAM("Created a new submap with id: "
                    << submap_id << ". Total submap number: "
                    << tsdf_submap_collection_ptr_->size());
  }
}

void TsdfSubmapServer::updateActiveSubmapMesh() {
  // NOTE(alexmillane): For the time being only the mesh from the currently
  // active submap is updated. This breaks down when the pose of past submaps is
  // changed. We will need to handle this separately later.
  active_submap_visualizer_ptr_->updateMeshLayer();
  // Getting the display mesh
  visualization_msgs::Marker marker;
  active_submap_visualizer_ptr_->getDisplayMesh(&marker);
  marker.header.frame_id = world_frame_;
  // Publishing
  active_submap_mesh_pub_.publish(marker);
}

bool TsdfSubmapServer::generateSeparatedMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& /*response*/) {  // NO LINT
  // Saving mesh to file if required
  if (!mesh_filename_.empty()) {
    // Getting the requested mesh type from the mesher
    voxblox::MeshLayer seperated_mesh_layer(
        tsdf_submap_collection_ptr_->block_size());
    submap_mesher_ptr_->generateSeparatedMesh(*tsdf_submap_collection_ptr_,
                                              &seperated_mesh_layer);
    bool success = outputMeshLayerAsPly(mesh_filename_, seperated_mesh_layer);
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
      return true;
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  } else {
    ROS_ERROR("No path to mesh specified in ros_params.");
  }
  return false;
}

bool TsdfSubmapServer::generateCombinedMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& /*response*/) {  // NO LINT
  // Saving mesh to file if required
  if (!mesh_filename_.empty()) {
    // Getting the requested mesh type from the mesher
    voxblox::MeshLayer combined_mesh_layer(
        tsdf_submap_collection_ptr_->block_size());
    submap_mesher_ptr_->generateCombinedMesh(*tsdf_submap_collection_ptr_,
                                             &combined_mesh_layer);
    bool success = outputMeshLayerAsPly(mesh_filename_, combined_mesh_layer);
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
      return true;
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  } else {
    ROS_ERROR("No path to mesh specified in ros_params.");
  }
  return false;
}

void TsdfSubmapServer::updateMeshEvent(const ros::TimerEvent& /*event*/) {
  if (mapIntialized()) {
    updateActiveSubmapMesh();
  }
}

void TsdfSubmapServer::visualizeSubMapBaseframes() const {
  // Get poses
  TransformationVector submap_poses;
  tsdf_submap_collection_ptr_->getSubMapPoses(&submap_poses);
  // Transform to message
  geometry_msgs::PoseArray pose_array_msg;
  posesToMsg(submap_poses, &pose_array_msg);
  pose_array_msg.header.frame_id = world_frame_;
  // Publish
  submap_poses_pub_.publish(pose_array_msg);
}

void TsdfSubmapServer::visualizeTrajectory() const {
  nav_msgs::Path path_msg;
  trajectory_visualizer_ptr_->getTrajectoryMsg(&path_msg);
  path_msg.header.frame_id = world_frame_;
  trajectory_pub_.publish(path_msg);
}

}  // namespace cblox
