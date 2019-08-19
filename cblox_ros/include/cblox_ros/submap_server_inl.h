#ifndef CBLOX_ROS_SUBMAP_SERVER_INL_H_
#define CBLOX_ROS_SUBMAP_SERVER_INL_H_

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <cblox_msgs/Submap.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <minkindr_conversions/kindr_msg.h>

#include <voxblox/utils/timing.h>
#include <voxblox_ros/ros_params.h>

#include <cblox/io/tsdf_submap_io.h>
#include "cblox_ros/pointcloud_conversions.h"
#include "cblox_ros/pose_vis.h"
#include "cblox_ros/ros_params.h"
#include "cblox_ros/submap_conversions.h"

#include "cblox_ros/submap_server.h"

namespace cblox {

template<typename SubmapType>
SubmapServer<SubmapType>::SubmapServer(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : SubmapServer<SubmapType>(
    nh, nh_private, getSubmapConfigFromRosParam<SubmapType>(nh_private),
    voxblox::getTsdfIntegratorConfigFromRosParam(nh_private),
    getTsdfIntegratorTypeFromRosParam(nh_private),
    voxblox::getMeshIntegratorConfigFromRosParam(nh_private)) {}

template<typename SubmapType>
SubmapServer<SubmapType>::SubmapServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const typename SubmapType::Config& submap_config,
    const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
    const voxblox::TsdfIntegratorType& tsdf_integrator_type,
    const voxblox::MeshIntegratorConfig& mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      transformer_(nh, nh_private),
      color_map_(new voxblox::GrayscaleColorMap()),
      num_integrated_frames_current_submap_(0),
      num_integrated_frames_per_submap_(kDefaultNumFramesPerSubmap) {
  ROS_DEBUG("Creating a TSDF Server");

  // Initial interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();

  // Creating the submap collection
  submap_collection_ptr_.reset(
      new SubmapCollection<SubmapType>(submap_config));

  // Creating an integrator and targetting the collection
  tsdf_submap_collection_integrator_ptr_.reset(
      new TsdfSubmapCollectionIntegrator(
          tsdf_integrator_config, tsdf_integrator_type, submap_collection_ptr_));

  // An object to visualize the submaps
  submap_mesher_ptr_.reset(new SubmapMesher(submap_config, mesh_config));
  active_submap_visualizer_ptr_.reset(
      new ActiveSubmapVisualizer(mesh_config, submap_collection_ptr_));

  // An object to visualize the trajectory
  trajectory_visualizer_ptr_.reset(new TrajectoryVisualizer);
}

template<typename SubmapType>
void SubmapServer<SubmapType>::subscribeToTopics() {
  // Subscribing to the input pointcloud
  int pointcloud_queue_size = kDefaultPointcloudQueueSize;
  nh_private_.param("pointcloud_queue_size", pointcloud_queue_size,
                    pointcloud_queue_size);
  pointcloud_sub_ = nh_.subscribe("pointcloud", pointcloud_queue_size,
                                  &SubmapServer<SubmapType>::pointcloudCallback, this);
  //
  int submap_queue_size = 20;
  submap_sub_ = nh_.subscribe("tsdf_submap_in", submap_queue_size,
                              &SubmapServer<SubmapType>::SubmapCallback, this);
  //
}

template<typename SubmapType>
void SubmapServer<SubmapType>::advertiseTopics() {
  // Services for saving meshes to file
  generate_separated_mesh_srv_ = nh_private_.advertiseService(
      "generate_separated_mesh",
      &SubmapServer<SubmapType>::generateSeparatedMeshCallback, this);
  generate_combined_mesh_srv_ = nh_private_.advertiseService(
      "generate_combined_mesh",
      &SubmapServer<SubmapType>::generateCombinedMeshCallback,
      this);
  // Services for loading and saving
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &SubmapServer<SubmapType>::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &SubmapServer<SubmapType>::loadMapCallback, this);
  // Real-time publishing for rviz
  active_submap_mesh_pub_ =
      nh_private_.advertise<visualization_msgs::Marker>("separated_mesh", 1);
  submap_poses_pub_ =
      nh_private_.advertise<geometry_msgs::PoseArray>("submap_baseframes", 1);
  trajectory_pub_ = nh_private_.advertise<nav_msgs::Path>("trajectory", 1);
  // Publisher for submaps
  submap_pub_ = nh_private_.advertise<cblox_msgs::MapLayer>("tsdf_submap_out", 1);
}

template<typename SubmapType>
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
    update_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
                                &SubmapServer<SubmapType>::updateMeshEvent, this);
  }
  // Frequency of submap creation
  nh_private_.param("num_integrated_frames_per_submap",
                    num_integrated_frames_per_submap_,
                    num_integrated_frames_per_submap_);
}

template<typename SubmapType>
void SubmapServer<SubmapType>::pointcloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  addMesageToPointcloudQueue(pointcloud_msg_in);
  // Processing messages in the queue
  servicePointcloudQueue();
}

template<typename SubmapType>
void SubmapServer<SubmapType>::addMesageToPointcloudQueue(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  if (pointcloud_msg_in->header.stamp - last_msg_time_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_ptcloud_ = pointcloud_msg_in->header.stamp;
    pointcloud_queue_.push(pointcloud_msg_in);
  }
}

template<typename SubmapType>
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
      createNewSubmap(T_G_C);
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

template<typename SubmapType>
bool SubmapServer<SubmapType>::getNextPointcloudFromQueue(
    std::queue <sensor_msgs::PointCloud2::Ptr> *queue,
    sensor_msgs::PointCloud2::Ptr *pointcloud_msg, Transformation *T_G_C) {
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

template<typename SubmapType>
void SubmapServer<SubmapType>::processPointCloudMessageAndInsert(
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
        (end - start).toSec(), submap_collection_ptr_->getActiveTsdfMap()
            .getTsdfLayer()
            .getNumberOfAllocatedBlocks(),
        num_integrated_frames_current_submap_);
  }
}

template<typename SubmapType>
void SubmapServer<SubmapType>::integratePointcloud(const Transformation& T_G_C,
                                                   const Pointcloud& ptcloud_C,
                                                   const Colors& colors,
                                                   const bool /*is_freespace_pointcloud*/) {
  // Note(alexmillane): Freespace pointcloud option left out for now.
  CHECK_EQ(ptcloud_C.size(), colors.size());
  tsdf_submap_collection_integrator_ptr_->integratePointCloud(T_G_C, ptcloud_C,
                                                              colors);
}

template<typename SubmapType>
void SubmapServer<SubmapType>::intializeMap(const Transformation& T_G_C) {
  // Just creates the first submap
  createNewSubmap(T_G_C);
}

template<typename SubmapType>
bool SubmapServer<SubmapType>::newSubmapRequired() const {
  return (num_integrated_frames_current_submap_ >
          num_integrated_frames_per_submap_);
}

template<typename SubmapType>
inline void SubmapServer<SubmapType>::finishSubmap() {
  if (submap_collection_ptr_->exists(
      submap_collection_ptr_->getActiveSubmapID())) {
    // publishing the old submap
    submap_collection_ptr_->getActiveSubmapPtr()->stopMappingTime();
    publishSubmap(submap_collection_ptr_->getActiveSubmapID());
    // generating ESDF map
    submap_collection_ptr_->getActiveSubmapPtr()->generateEsdf();
  }
}

template<>
inline void SubmapServer<TsdfSubmap>::finishSubmap() {
  if (submap_collection_ptr_->exists(
      submap_collection_ptr_->getActiveSubmapID())) {
    // publishing the old submap
    submap_collection_ptr_->getActiveSubmapPtr()->stopMappingTime();
    publishSubmap(submap_collection_ptr_->getActiveSubmapID());
  }
}

template<typename SubmapType>
void SubmapServer<SubmapType>::createNewSubmap(const Transformation& T_G_C) {
  // finishing up the last submap
  finishSubmap();

  // Creating the submap
  const SubmapID submap_id =
      submap_collection_ptr_->createNewSubmap(T_G_C);
  // Activating the submap in the frame integrator
  tsdf_submap_collection_integrator_ptr_->switchToActiveSubmap();
  // Resetting current submap counters
  num_integrated_frames_current_submap_ = 0;

  // Updating the active submap mesher
  active_submap_visualizer_ptr_->switchToActiveSubmap();

  // Publish the baseframes
  visualizeSubmapBaseframes();

  // Time the start of recording
  submap_collection_ptr_->getActiveSubmapPtr()->startMappingTime();

  if (verbose_) {
    ROS_INFO_STREAM("Created a new submap with id: "
                        << submap_id << ". Total submap number: "
                        << submap_collection_ptr_->size());
  }
}

template<typename SubmapType>
void SubmapServer<SubmapType>::visualizeActiveSubmapMesh() {
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

template<typename SubmapType>
void SubmapServer<SubmapType>::visualizeWholeMap() {
  // Looping through the whole map, meshing and publishing.
  ROS_INFO("[CbloxServer] visualizing submaps (#%lu)", submap_collection_ptr_->size());
  for (const SubmapID submap_id : submap_collection_ptr_->getIDs()) {
    submap_collection_ptr_->activateSubmap(submap_id);
    active_submap_visualizer_ptr_->switchToActiveSubmap();
    visualizeActiveSubmapMesh();
  }
}

template<typename SubmapType>
bool SubmapServer<SubmapType>::generateSeparatedMeshCallback(
    std_srvs::Empty::Request&  /*request*/,
    std_srvs::Empty::Response&  /*response*/) {  // NO LINT
  // Saving mesh to file if required
  if (!mesh_filename_.empty()) {
    // Getting the requested mesh type from the mesher
    voxblox::MeshLayer seperated_mesh_layer(
        submap_collection_ptr_->block_size());
    submap_mesher_ptr_->generateSeparatedMesh(*submap_collection_ptr_,
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

template<typename SubmapType>
bool SubmapServer<SubmapType>::generateCombinedMeshCallback(
    std_srvs::Empty::Request&  /*request*/,
    std_srvs::Empty::Response&  /*response*/) {  // NO LINT
  // Saving mesh to file if required
  if (!mesh_filename_.empty()) {
    // Getting the requested mesh type from the mesher
    voxblox::MeshLayer combined_mesh_layer(
        submap_collection_ptr_->block_size());
    submap_mesher_ptr_->generateCombinedMesh(*submap_collection_ptr_,
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

template<typename SubmapType>
void SubmapServer<SubmapType>::updateMeshEvent(const ros::TimerEvent&  /*event*/) {
  if (mapIntialized()) {
    visualizeActiveSubmapMesh();
  }
}

template<typename SubmapType>
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

template<typename SubmapType>
void SubmapServer<SubmapType>::visualizeTrajectory() const {
  nav_msgs::Path path_msg;
  trajectory_visualizer_ptr_->getTrajectoryMsg(&path_msg);
  path_msg.header.frame_id = world_frame_;
  trajectory_pub_.publish(path_msg);
}

template<typename SubmapType>
bool SubmapServer<SubmapType>::saveMap(const std::string& file_path) {
  return io::SaveSubmapCollection(*submap_collection_ptr_,
                                         file_path);
}

template<typename SubmapType>
bool SubmapServer<SubmapType>::loadMap(const std::string& file_path) {
  ROS_INFO("[CbloxServer] loading map");
  bool success = io::LoadSubmapCollection<SubmapType>(
      file_path,& submap_collection_ptr_);
  if (!success) {
    return false;
  }

  ROS_INFO("[CbloxServer] Successfully loaded TSDFSubmapCollection.");
  constexpr bool kVisualizeMapOnLoad = true;
  if (kVisualizeMapOnLoad) {
    ROS_INFO("[CbloxServer] Publishing loaded map's mesh.");
    visualizeWholeMap();
  }
  constexpr bool kPublishMapOnLoad = true;
  if (kPublishMapOnLoad) {
    ROS_INFO("[CbloxServer] Publishing loaded submaps.");
    publishWholeMap();
  }

  return true;
}

template<typename SubmapType>
bool SubmapServer<SubmapType>::saveMapCallback(voxblox_msgs::FilePath::Request& request,
                                               voxblox_msgs::FilePath::Response& /*response*/) {
  return saveMap(request.file_path);
}

template<typename SubmapType>
bool SubmapServer<SubmapType>::loadMapCallback(voxblox_msgs::FilePath::Request& request,
                                               voxblox_msgs::FilePath::Response& /*response*/) {
  bool success = loadMap(request.file_path);
  return success;
}

template<>
inline const SubmapCollection<TsdfSubmap>::Ptr
SubmapServer<TsdfSubmap>::getSubmapCollectionPtr() const {
  return submap_collection_ptr_;
}

template<>
inline const SubmapCollection<TsdfEsdfSubmap>::Ptr
SubmapServer<TsdfEsdfSubmap>::getSubmapCollectionPtr() const {
  return submap_collection_ptr_;
}

template<typename SubmapType>
void SubmapServer<SubmapType>::publishSubmap(
    SubmapID submap_id, bool global_map) const {
  if (submap_pub_.getNumSubscribers() == 0) {
    ROS_WARN("[CbloxServer] no subscribers to submaps");
    return;
  }
  if (!submap_collection_ptr_->getSubmapPtr(submap_id)) {
    ROS_WARN("[CbloxServer] could not get submap pointer");
    return;
  }
  if (verbose_) {
    ROS_INFO("[CbloxServer] publishing submap %d", submap_id);
  }

  // set timer
  timing::Timer publish_map_timer("cblox/0 - publish map");

  cblox_msgs::MapLayer submap_msg;
  if (global_map) {
    // TODO: analyze necessity of this case
    // Merge submaps to global TSDF map
    Transformation T_M_S;
    submap_id = 0;
    timing::Timer get_global_timer("cblox/1 - get global map");
    voxblox::TsdfMap::Ptr tsdf_map =
        submap_collection_ptr_->getProjectedMap();
    get_global_timer.Stop();

    timing::Timer make_dummy_timer("cblox/2 - make dummy submap");
    TsdfSubmap::Ptr submap_ptr(new TsdfSubmap(T_M_S, submap_id,
                                              submap_collection_ptr_->getConfig()));
    // TODO: switch from copy to moving layer
    submap_ptr->getTsdfMapPtr().reset(
        new voxblox::TsdfMap(tsdf_map->getTsdfLayer()));
    make_dummy_timer.Stop();

    // serialize into message
    timing::Timer serialize_timer("cblox/3 - serialize");
    submap_msg.map_header.is_submap = false;
    serializeSubmapToMsg<TsdfSubmap>(submap_ptr, &submap_msg);
    serialize_timer.Stop();
  } else {
    // Get submap for publishing
    TsdfSubmap::Ptr submap_ptr = submap_collection_ptr_->
        getSubmapPtr(submap_id);
    timing::Timer serialize_timer("cblox/3 - serialize");
    serializeSubmapToMsg<TsdfSubmap>(submap_ptr, &submap_msg);
    serialize_timer.Stop();
  }

  // Publish message
  timing::Timer publish_timer("cblox/4 - publish");
  submap_pub_.publish(submap_msg);
  publish_timer.Stop();

  // stop timer
  publish_map_timer.Stop();
}

template<typename SubmapType>
void SubmapServer<SubmapType>::SubmapCallback(
    const cblox_msgs::MapLayerPtr& msg_in) {
  timing::Timer read_map_timer("cblox/receive submap");

  // push newest message in queue to service
  submap_queue_.push(msg_in);
  // service message in queue
  deserializeMsgToSubmap<SubmapType>(submap_queue_.front().get(),
      getSubmapCollectionPtr());
  submap_queue_.pop();

  read_map_timer.Stop();
}

template <typename SubmapType>
void SubmapServer<SubmapType>::publishWholeMap() const {
  for (const SubmapID submap_id : submap_collection_ptr_->getIDs()) {
    publishSubmap(submap_id);
    ros::Duration sleepy(0.25);
    sleepy.sleep();
  }
}

} // namespace cblox
#endif  // CBLOX_ROS_SUBMAP_SERVER_INL_H_