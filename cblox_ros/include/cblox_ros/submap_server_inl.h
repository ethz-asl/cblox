#ifndef CBLOX_ROS_SUBMAP_SERVER_INL_H_
#define CBLOX_ROS_SUBMAP_SERVER_INL_H_

#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/PoseArray.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Path.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <voxblox/utils/planning_utils.h>
#include <voxblox/utils/timing.h>
#include <voxblox_ros/ros_params.h>

#include <cblox/io/submap_io.h>
#include <cblox_msgs/SubmapSrv.h>

#include "cblox_ros/pointcloud_conversions.h"
#include "cblox_ros/pose_vis.h"
#include "cblox_ros/ros_params.h"
#include "cblox_ros/submap_conversions.h"

namespace cblox {

template <typename SubmapType>
SubmapServer<SubmapType>::SubmapServer(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : SubmapServer<SubmapType>(
          nh, nh_private, getSubmapConfigFromRosParam<SubmapType>(nh_private),
          voxblox::getTsdfIntegratorConfigFromRosParam(nh_private),
          getTsdfIntegratorTypeFromRosParam(nh_private),
          voxblox::getMeshIntegratorConfigFromRosParam(nh_private)) {}

template <typename SubmapType>
SubmapServer<SubmapType>::SubmapServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const typename SubmapType::Config& submap_config,
    const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
    const voxblox::TsdfIntegratorType& tsdf_integrator_type,
    const voxblox::MeshIntegratorConfig& mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(false),
      world_frame_("world"),
      transformer_(nh, nh_private),
      color_map_(new voxblox::GrayscaleColorMap()),
      num_integrated_frames_current_submap_(0),
      num_integrated_frames_per_submap_(kDefaultNumFramesPerSubmap),
      truncation_distance_(tsdf_integrator_config.default_truncation_distance) {
  ROS_DEBUG("Creating a TSDF Server");

  // Initial interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();

  // Creating the submap collection
  submap_collection_ptr_.reset(new SubmapCollection<SubmapType>(submap_config));

  // Creating an integrator and targetting the collection
  tsdf_submap_collection_integrator_ptr_.reset(
      new TsdfSubmapCollectionIntegrator(tsdf_integrator_config,
                                         tsdf_integrator_type,
                                         submap_collection_ptr_));

  // An object to visualize the submaps
  submap_mesher_ptr_.reset(new SubmapMesher(submap_config, mesh_config));
  std::lock_guard<std::mutex> lock(visualizer_mutex_);
  active_submap_visualizer_ptr_.reset(
      new ActiveSubmapVisualizer(mesh_config, submap_collection_ptr_));
  active_submap_visualizer_ptr_->setVerbose(verbose_);
  float opacity = 1.0;
  nh_private_.param("mesh_opacity", opacity, opacity);
  active_submap_visualizer_ptr_->setOpacity(opacity);

  // An object to visualize the trajectory
  trajectory_visualizer_ptr_.reset(new TrajectoryVisualizer);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::subscribeToTopics() {
  // Subscribing to the input pointcloud
  int pointcloud_queue_size = kDefaultPointcloudQueueSize;
  nh_private_.param("pointcloud_queue_size", pointcloud_queue_size,
                    pointcloud_queue_size);
  pointcloud_sub_ =
      nh_.subscribe("pointcloud", pointcloud_queue_size,
                    &SubmapServer<SubmapType>::pointcloudCallback, this);
  // Subscribing to new submaps and submap pose updates
  constexpr int kQueueSize = 20;
  submap_sub_ = nh_.subscribe("submap_in", kQueueSize,
                              &SubmapServer<SubmapType>::SubmapCallback, this);
  pose_sub_ = nh_.subscribe("submap_pose_in", kQueueSize,
                            &SubmapServer::PoseCallback, this);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::advertiseTopics() {
  // Services for saving meshes to file
  generate_separated_mesh_srv_ = nh_private_.advertiseService(
      "generate_separated_mesh",
      &SubmapServer<SubmapType>::generateSeparatedMeshCallback, this);
  generate_combined_mesh_srv_ = nh_private_.advertiseService(
      "generate_combined_mesh",
      &SubmapServer<SubmapType>::generateCombinedMeshCallback, this);
  // Services for loading and saving
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &SubmapServer<SubmapType>::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &SubmapServer<SubmapType>::loadMapCallback, this);
  // Real-time publishing for rviz
  active_submap_mesh_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("separated_mesh",
                                                             1);
  submap_poses_pub_ =
      nh_private_.advertise<geometry_msgs::PoseArray>("submap_baseframes", 1);
  trajectory_pub_ = nh_private_.advertise<nav_msgs::Path>("trajectory", 1);

  // Publisher for submaps
  pose_pub_ =
      nh_private_.advertise<cblox_msgs::MapPoseUpdates>("submap_pose_out", 1);
  submap_pub_ =
      nh_private_.advertise<cblox_msgs::MapLayer>("tsdf_submap_out", 1);
  sdf_slice_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("sdf_slice", 1);
  // Service for submaps
  publish_active_submap_srv_ = nh_private_.advertiseService(
      "publish_active_submap",
      &SubmapServer<SubmapType>::publishActiveSubmapCallback, this);
  publish_submap_poses_srv_ = nh_private_.advertiseService(
      "publish_submap_poses",
      &SubmapServer<SubmapType>::publishSubmapPosesCallback, this);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::getParametersFromRos() {
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
    update_mesh_timer_ = nh_private_.createTimer(
        ros::Duration(update_mesh_every_n_sec),
        &SubmapServer<SubmapType>::updateMeshEvent, this);
  }
  // Frequency of submap creation
  nh_private_.param("num_integrated_frames_per_submap",
                    num_integrated_frames_per_submap_,
                    num_integrated_frames_per_submap_);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::pointcloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  addMesageToPointcloudQueue(pointcloud_msg_in);
  // Processing messages in the queue
  servicePointcloudQueue();
}

template <typename SubmapType>
void SubmapServer<SubmapType>::addMesageToPointcloudQueue(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  if (pointcloud_msg_in->header.stamp - last_msg_time_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_ptcloud_ = pointcloud_msg_in->header.stamp;
    pointcloud_queue_.push(pointcloud_msg_in);
  }
}

template <typename SubmapType>
void SubmapServer<SubmapType>::servicePointcloudQueue() {
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
      createNewSubmap(T_G_C, pointcloud_msg->header.stamp);
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

template <typename SubmapType>
bool SubmapServer<SubmapType>::getNextPointcloudFromQueue(
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

template <typename SubmapType>
void SubmapServer<SubmapType>::processPointCloudMessageAndInsert(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  // Convert the PCL pointcloud into our awesome format.
  Pointcloud points_C;
  Colors colors;
  convertPointcloudMsg(*color_map_, pointcloud_msg, &points_C, &colors);

  if (verbose_) {
    ROS_INFO("[CbloxServer] Integrating a pointcloud with %lu points.",
             points_C.size());
  }

  if (!mapIntialized()) {
    ROS_INFO("[CbloxServer] Initializing map.");
    intializeMap(T_G_C);
  }

  ros::WallTime start = ros::WallTime::now();
  integratePointcloud(T_G_C, points_C, colors, is_freespace_pointcloud);
  ros::WallTime end = ros::WallTime::now();
  num_integrated_frames_current_submap_++;
  if (verbose_) {
    ROS_INFO(
        "[CbloxServer] Finished integrating in %f seconds, have %lu blocks. "
        "%u frames integrated to current submap.",
        (end - start).toSec(),
        submap_collection_ptr_->getActiveTsdfMap()
            .getTsdfLayer()
            .getNumberOfAllocatedBlocks(),
        num_integrated_frames_current_submap_);
  }
}

template <typename SubmapType>
void SubmapServer<SubmapType>::integratePointcloud(
    const Transformation& T_G_C, const Pointcloud& ptcloud_C,
    const Colors& colors, const bool /*is_freespace_pointcloud*/) {
  // Note(alexmillane): Freespace pointcloud option left out for now.
  CHECK_EQ(ptcloud_C.size(), colors.size());
  tsdf_submap_collection_integrator_ptr_->integratePointCloud(T_G_C, ptcloud_C,
                                                              colors);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::intializeMap(const Transformation& T_G_C) {
  // Just creates the first submap
  createNewSubmap(T_G_C, ros::Time::now());
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::newSubmapRequired() const {
  return (num_integrated_frames_current_submap_ >
          num_integrated_frames_per_submap_);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::createNewSubmap(const Transformation& T_G_C,
                                               const ros::Time& timestamp) {
  // finishing up the last submap
  if (!submap_collection_ptr_->empty()) {
    std::thread finish_submap_thread(
        &SubmapServer<SubmapType>::finishSubmap, this,
        submap_collection_ptr_->getActiveSubmapID());
    finish_submap_thread.detach();
  }

  // Creating the submap
  const SubmapID submap_id = submap_collection_ptr_->createNewSubmap(T_G_C);
  // Activating the submap in the frame integrator
  tsdf_submap_collection_integrator_ptr_->switchToActiveSubmap();
  // Resetting current submap counters
  num_integrated_frames_current_submap_ = 0;

  // Updating the active submap mesher
  active_submap_visualizer_ptr_->switchToActiveSubmap();

  // Publish the baseframes
  visualizeSubmapBaseframes();

  // Time the start of recording
  submap_collection_ptr_->getActiveSubmapPtr()->startMappingTime(
      timestamp.toNSec());

  if (verbose_) {
    ROS_INFO_STREAM("Created a new submap with id: "
                    << submap_id << ". Total submap number: "
                    << submap_collection_ptr_->size());
  }
}

template <typename SubmapType>
void SubmapServer<SubmapType>::finishSubmap(const SubmapID submap_id) {
  if (submap_collection_ptr_->exists(submap_id)) {
    typename SubmapType::Ptr submap_ptr =
        submap_collection_ptr_->getSubmapPtr(submap_id);
    // Stopping the mapping interval.
    submap_ptr->stopMappingTime(ros::Time::now().toNSec());
    // Finish submap.
    submap_ptr->finishSubmap();
    // publishing the old submap
    publishSubmap(submap_id);
    ROS_INFO("[CbloxServer] Finished submap %d", submap_id);
  }
}

template <typename SubmapType>
void SubmapServer<SubmapType>::visualizeSubmapMesh(const SubmapID submap_id) {
  if (active_submap_mesh_pub_.getNumSubscribers() < 1) {
    return;
  }

  if (submap_collection_ptr_->getSubmapPtr(submap_id)
          ->getTsdfMapPtr()
          ->getTsdfLayerPtr()
          ->getNumberOfAllocatedBlocks() == 0) {
    ROS_WARN("[CbloxServer] Submap %d has no allocated blocks yet to visualize",
             submap_id);
    return;
  }

  std::lock_guard<std::mutex> lock(visualizer_mutex_);
  active_submap_visualizer_ptr_->switchToSubmap(submap_id);
  active_submap_visualizer_ptr_->updateMeshLayer();
  // Getting the display mesh
  visualization_msgs::Marker marker;
  active_submap_visualizer_ptr_->getDisplayMesh(&marker);
  marker.header.frame_id = world_frame_;
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  // Publishing
  active_submap_mesh_pub_.publish(marker_array);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::visualizeWholeMap() {
  // Looping through the whole map, meshing and publishing.
  for (const SubmapID submap_id : submap_collection_ptr_->getIDs()) {
    visualizeSubmapMesh(submap_id);
  }
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::generateSeparatedMeshCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {  // NO LINT
  // Saving mesh to file if required
  if (!mesh_filename_.empty()) {
    // Getting the requested mesh type from the mesher
    voxblox::MeshLayer seperated_mesh_layer(
        submap_collection_ptr_->block_size());
    submap_mesher_ptr_->generateSeparatedMesh(*submap_collection_ptr_,
                                              &seperated_mesh_layer);
    bool success = outputMeshLayerAsPly(mesh_filename_, seperated_mesh_layer);
    if (success) {
      ROS_INFO("[CbloxServer] Output file as PLY: %s", mesh_filename_.c_str());
      return true;
    } else {
      ROS_INFO("[CbloxServer] Failed to output mesh as PLY: %s",
               mesh_filename_.c_str());
    }
  } else {
    ROS_ERROR("[CbloxServer] No path to mesh specified in ros_params.");
  }
  return false;
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::generateCombinedMeshCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {  // NO LINT
  // Saving mesh to file if required
  if (!mesh_filename_.empty()) {
    // Getting the requested mesh type from the mesher
    voxblox::MeshLayer combined_mesh_layer(
        submap_collection_ptr_->block_size());
    submap_mesher_ptr_->generateCombinedMesh(*submap_collection_ptr_,
                                             &combined_mesh_layer);
    bool success = outputMeshLayerAsPly(mesh_filename_, combined_mesh_layer);
    if (success) {
      ROS_INFO("[CbloxServer] Output file as PLY: %s", mesh_filename_.c_str());
      return true;
    } else {
      ROS_INFO("[CbloxServer] Failed to output mesh as PLY: %s",
               mesh_filename_.c_str());
    }
  } else {
    ROS_ERROR("[CbloxServer] No path to mesh specified in ros_params.");
  }
  return false;
}

template <typename SubmapType>
void SubmapServer<SubmapType>::updateMeshEvent(
    const ros::TimerEvent& /*event*/) {
  if (mapIntialized()) {
    visualizeSubmapMesh(submap_collection_ptr_->getActiveSubmapID());
    visualizeSlice(submap_collection_ptr_->getActiveSubmapID());
  }
}

template <typename SubmapType>
void SubmapServer<SubmapType>::visualizeSubmapBaseframes() const {
  // Get poses
  TransformationVector submap_poses;
  submap_collection_ptr_->getSubmapPoses(&submap_poses);
  // Transform to message
  geometry_msgs::PoseArray pose_array_msg;
  posesToMsg(submap_poses, &pose_array_msg);
  pose_array_msg.header.frame_id = world_frame_;
  // Publish
  submap_poses_pub_.publish(pose_array_msg);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::visualizeTrajectory() const {
  nav_msgs::Path path_msg;
  CHECK(trajectory_visualizer_ptr_);
  trajectory_visualizer_ptr_->getTrajectoryMsg(&path_msg);
  path_msg.header.frame_id = world_frame_;
  trajectory_pub_.publish(path_msg);
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::saveMap(const std::string& file_path) {
  return io::SaveSubmapCollection(*submap_collection_ptr_, file_path);
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::loadMap(const std::string& file_path) {
  bool success =
      io::LoadSubmapCollection<SubmapType>(file_path, &submap_collection_ptr_);
  if (success) {
    ROS_INFO("[CbloxServer] Successfully loaded SubmapCollection.");
    constexpr bool kVisualizeMapOnLoad = true;
    if (kVisualizeMapOnLoad) {
      ROS_INFO("[CbloxServer] Publishing loaded map's mesh.");
      visualizeWholeMap();
    }
  }
  return success;
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::saveMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& /*response*/) {
  return saveMap(request.file_path);
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::loadMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& /*response*/) {
  bool success = loadMap(request.file_path);
  return success;
}

template <typename SubmapType>
void SubmapServer<SubmapType>::publishSubmap(SubmapID submap_id) const {
  if (submap_pub_.getNumSubscribers() == 0) {
    return;
  }
  if (verbose_) {
    ROS_INFO("[CbloxServer] Publishing submap %d", submap_id);
  }

  CHECK(submap_collection_ptr_->exists(submap_id))
      << "[CbloxServer] Submap " << submap_id << " does not exist!";
  cblox_msgs::MapLayer submap_msg;
  serializeSubmapToMsg<SubmapType>(submap_collection_ptr_->getSubmap(submap_id),
                                   &submap_msg);
  submap_pub_.publish(submap_msg);
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::publishActiveSubmapCallback(
    cblox_msgs::SubmapSrvRequest& /*request*/,
    cblox_msgs::SubmapSrvResponse& response) {
  std::lock_guard<std::mutex> lock(visualizer_mutex_);

  SubmapID submap_id = submap_collection_ptr_->getActiveSubmapID();
  if (!submap_collection_ptr_->exists(submap_id)) {
    ROS_ERROR("[CbloxServer] Active submap does not exist");
    return false;
  }
  const TsdfSubmap& submap = submap_collection_ptr_->getSubmap(submap_id);
  if (submap.getTsdfMap().getTsdfLayer().getNumberOfAllocatedBlocks() == 0) {
    if (verbose_) {
      ROS_WARN("[CbloxServer] Active submap has no allocated blocks yet");
    }
    return false;
  }

  cblox_msgs::MapLayer submap_msg;
  serializeSubmapToMsg<TsdfSubmap>(submap, &submap_msg);
  response.submap_msg = submap_msg;
  return true;
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::publishActiveSubmap() {
  if (submap_collection_ptr_->empty()) {
    ROS_WARN("[CbloxPlanner] Active submap does not exist!");
    return false;
  }
  SubmapID submap_id = submap_collection_ptr_->getActiveSubmapID();
  submap_collection_ptr_->getSubmapPtr(submap_id)->prepareForPublish();
  publishSubmap(submap_id);
  return true;
}

template <typename SubmapType>
void SubmapServer<SubmapType>::publishSubmapPoses() const {
  if (pose_pub_.getNumSubscribers() == 0) {
    return;
  }
  if (verbose_) {
    ROS_INFO("[CbloxServer] Publishing submap poses");
  }

  cblox_msgs::MapPoseUpdates pose_msg;
  pose_msg.header.frame_id = world_frame_;
  pose_msg.header.stamp = ros::Time::now();
  for (SubmapID submap_id : submap_collection_ptr_->getIDs()) {
    const SubmapType& submap = submap_collection_ptr_->getSubmap(submap_id);
    pose_msg.map_headers.emplace_back(
        generateSubmapHeaderMsg<SubmapType>(submap));
  }
  pose_pub_.publish(pose_msg);
}

template <typename SubmapType>
void SubmapServer<SubmapType>::PoseCallback(
    const cblox_msgs::MapPoseUpdates& msg) {
  if (verbose_) {
    ROS_INFO("[CbloxServer] Received pose update");
  }
  std::thread process_thread(&SubmapServer<SubmapType>::processPoseUpdate, this,
                             msg);
  process_thread.detach();
}

template <typename SubmapType>
void SubmapServer<SubmapType>::processPoseUpdate(
    const cblox_msgs::MapPoseUpdates& msg) {
  {
    std::lock_guard<std::mutex> lock(visualizer_mutex_);
    SubmapIdPoseMap id_pose_map;
    deserializeMsgToPose(msg, &id_pose_map);
    submap_collection_ptr_->setSubmapPoses(id_pose_map);
  }

  visualizeWholeMap();
}

template <typename SubmapType>
void SubmapServer<SubmapType>::SubmapCallback(
    const cblox_msgs::MapLayerPtr& msg_in) {
  // push newest message in queue to service
  if (verbose_) {
    ROS_INFO("[CbloxPlanner] Received submap msg");
  }
  submap_queue_.push(msg_in);

  // service message in queue
  if (!submap_queue_.empty()) {
    SubmapID submap_id = deserializeMsgToSubmap<SubmapType>(
        submap_queue_.front().get(), getSubmapCollectionPtr());
    submap_queue_.pop();
    visualizeSubmapMesh(submap_id);
  }
}

template <typename SubmapType>
bool SubmapServer<SubmapType>::publishSubmapPosesCallback(
    std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) {
  publishSubmapPoses();
  return true;
}

template <typename SubmapType>
void SubmapServer<SubmapType>::publishWholeMap() const {
  for (const SubmapID submap_id : submap_collection_ptr_->getIDs()) {
    publishSubmap(submap_id);
    // delay to allow for processing of sent maps
    ros::Duration sleepy(0.1);
    sleepy.sleep();
  }
}

template <typename SubmapType>
void SubmapServer<SubmapType>::visualizeSlice(const SubmapID submap_id) const {
  if (sdf_slice_pub_.getNumSubscribers() < 1) {
    return;
  }
  if (!submap_collection_ptr_->exists(submap_id)) {
    ROS_WARN("[CbloxServer] Submap %d does not exist.", submap_id);
    return;
  }

  if (verbose_) {
    ROS_INFO("[CbloxServer] Visualizing ESDF slice of submap %d at height %.2f",
             submap_id, slice_height_);
  }

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.frame_id = world_frame_;
  vertex_marker.ns = std::string("esdf_slice_") + std::to_string(submap_id);
  vertex_marker.ns = "esdf_slice";
  vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
  Transformation pose =
      submap_collection_ptr_->getSubmapPtr(submap_id)->getPose();
  vertex_marker.pose.orientation.w = 1.0;
  vertex_marker.scale.x =
      submap_collection_ptr_->getActiveTsdfMapPtr()->voxel_size();
  vertex_marker.scale.y = vertex_marker.scale.x;
  vertex_marker.scale.z = vertex_marker.scale.x;
  geometry_msgs::Point point_msg;
  std_msgs::ColorRGBA color_msg;
  color_msg.r = 0.0;
  color_msg.g = 0.0;
  color_msg.b = 0.0;
  color_msg.a = 1.0;

  // find maximum distance in SDF
  float max_dist = 0;
  typename SubmapType::Ptr submap_ptr =
      submap_collection_ptr_->getSubmapPtr(submap_id);
  voxblox::Layer<EsdfVoxel>* layer =
      submap_ptr->getEsdfMapPtr()->getEsdfLayerPtr();
  voxblox::BlockIndexList block_list;
  layer->getAllAllocatedBlocks(&block_list);

  for (const voxblox::BlockIndex& block_id : block_list) {
    if (!layer->hasBlock(block_id)) continue;
    voxblox::Block<EsdfVoxel>::Ptr block = layer->getBlockPtrByIndex(block_id);
    for (size_t voxel_id = 0; voxel_id < block->num_voxels(); voxel_id++) {
      const EsdfVoxel& voxel = block->getVoxelByLinearIndex(voxel_id);
      max_dist = std::max(max_dist, voxel.distance);
    }
  }

  // visualize SDF as colored voxel markers
  int block_num = 0;
  for (const voxblox::BlockIndex& block_id : block_list) {
    if (!layer->hasBlock(block_id)) continue;
    voxblox::Block<EsdfVoxel>::Ptr block = layer->getBlockPtrByIndex(block_id);
    for (size_t voxel_id = 0; voxel_id < block->num_voxels(); voxel_id++) {
      const EsdfVoxel& voxel = block->getVoxelByLinearIndex(voxel_id);
      voxblox::Point position =
          submap_ptr->getPose() *
          block->computeCoordinatesFromLinearIndex(voxel_id);

      if (!voxel.observed) {
        continue;
      }

      color_msg.r = 0.0;
      color_msg.g = 0.0;
      if (voxel.observed) {
        color_msg.r = std::max(
            std::min((max_dist - voxel.distance) / 2.0 / max_dist, 1.0), 0.0);
        color_msg.g = std::max(
            std::min((max_dist + voxel.distance) / 2.0 / max_dist, 1.0), 0.0);
      }

      if (std::abs(position.z() - slice_height_) <
          submap_ptr->getEsdfMapPtr()->voxel_size() / 2) {
        vertex_marker.id =
            block_num +
            voxel_id * std::pow(10, std::round(std::log10(block_list.size())));
        tf::pointEigenToMsg(position.cast<double>(), point_msg);

        vertex_marker.points.push_back(point_msg);
        vertex_marker.colors.push_back(color_msg);
      }
    }
    block_num++;
  }

  marker_array.markers.push_back(vertex_marker);
  sdf_slice_pub_.publish(marker_array);
}
template <>
void SubmapServer<TsdfSubmap>::visualizeSlice(const SubmapID submap_id) const;

template <typename SubmapType>
TsdfMap::Ptr SubmapServer<SubmapType>::projectAndVisualizeIteratively() {
  // prep global map
  TsdfMap::Ptr projected_tsdf_map_ptr =
      std::make_shared<TsdfMap>(submap_collection_ptr_->getConfig());
  Layer<voxblox::TsdfVoxel>* combined_tsdf_layer_ptr =
      projected_tsdf_map_ptr->getTsdfLayerPtr();
  // prep visualizing
  voxblox::MeshIntegratorConfig mesh_config =
      voxblox::getMeshIntegratorConfigFromRosParam(nh_private_);

  // Looping over the current submaps
  for (const SubmapID submap_id : submap_collection_ptr_->getIDs()) {
    // Getting the tsdf submap and its pose
    const TsdfMap& tsdf_map =
        submap_collection_ptr_->getSubmapPtr(submap_id)->getTsdfMap();
    const Transformation& T_G_S =
        submap_collection_ptr_->getSubmapPtr(submap_id)->getPose();
    // Merging layers the submap into the global layer
    mergeLayerAintoLayerB(tsdf_map.getTsdfLayer(), T_G_S,
                          combined_tsdf_layer_ptr);

    // visualize
    voxblox::MeshLayer::Ptr mesh_layer;
    mesh_layer.reset(
        new voxblox::MeshLayer(combined_tsdf_layer_ptr->block_size()));
    voxblox::MeshIntegrator<TsdfVoxel> mesh_integrator(
        mesh_config, combined_tsdf_layer_ptr, mesh_layer.get());
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);

    visualization_msgs::Marker marker;
    SubmapMesher::colorMeshLayer(voxblox::Color::Gray(), mesh_layer.get());
    const voxblox::ColorMode color_mode = voxblox::ColorMode::kLambertColor;
    voxblox::fillMarkerWithMesh(mesh_layer, color_mode, &marker);
    marker.id = submap_id;
    marker.header.frame_id = world_frame_;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    active_submap_mesh_pub_.publish(marker_array);
  }

  // Returning the new map
  return projected_tsdf_map_ptr;
}

}  // namespace cblox
#endif  // CBLOX_ROS_SUBMAP_SERVER_INL_H_
