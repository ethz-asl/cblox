#ifndef CBLOX_ROS_SUBMAP_SERVER_H_
#define CBLOX_ROS_SUBMAP_SERVER_H_

#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_ros/transformer.h>

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/core/tsdf_submap.h>
#include <cblox/integrator/tsdf_submap_collection_integrator.h>
#include <cblox/mesh/submap_mesher.h>
#include <cblox_msgs/Submap.h>
#include <cblox_msgs/MapLayer.h>
#include <cblox_msgs/MapPoseUpdate.h>

#include "cblox_ros/active_submap_visualizer.h"
#include "cblox_ros/trajectory_visualizer.h"

namespace cblox {

// Default values for parameters
constexpr bool kDefaultVerbose = true;
constexpr int kDefaultNumFramesPerSubmap = 20;
constexpr double kDefaultMinTimeBetweenMsgsSec = 0.0;

// Data queue sizes
constexpr int kDefaultPointcloudQueueSize = 1;

// Receives ROS Data and produces a collection of submaps
template <typename SubmapType>
class SubmapServer {
 public:
  // Constructor
  explicit SubmapServer(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);
  SubmapServer(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      const typename SubmapType::Config& submap_config,
      const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
      const voxblox::TsdfIntegratorType& tsdf_integrator_type,
      const voxblox::MeshIntegratorConfig& mesh_config);
  virtual ~SubmapServer() {}

  // Pointcloud data subscriber
  virtual void pointcloudCallback(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);

  // Saving and Loading callbacks
  bool saveMap(const std::string& file_path);
  bool loadMap(const std::string& file_path);
  bool saveMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT

  // Update the mesh and publish for visualization
  void updateMeshEvent(const ros::TimerEvent& /*event*/);

  // Access Submap Collection Pointer
  inline const typename SubmapCollection<SubmapType>::Ptr&
      getSubmapCollectionPtr() const;

  // Visualize mesh
  void visualizeActiveSubmapMesh();
  void visualizeSubmapMesh(const SubmapID& submap_id);
  void visualizeWholeMap();

  // Visualize trajectory
  void visualizeSubmapBaseframes() const;
  void visualizeTrajectory() const;

  // Mesh output
  bool generateSeparatedMeshCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
  bool generateCombinedMeshCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

 protected:
  // Gets parameters
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // The two actions on pointcloud callback; add message to queue for
  // processing, and process messages in the queue.
  void addMesageToPointcloudQueue(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in);
  void servicePointcloudQueue();

  // Checks if we can get the next message from queue.
  bool getNextPointcloudFromQueue(
      std::queue<sensor_msgs::PointCloud2::Ptr>* queue,
      sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C);

  // Pointcloud integration
  void processPointCloudMessageAndInsert(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
      const Transformation& T_G_C, const bool is_freespace_pointcloud);
  void integratePointcloud(const Transformation& T_G_C,
                           const Pointcloud& ptcloud_C, const Colors& colors,
                           const bool is_freespace_pointcloud);

  // Initializes the map
  bool mapIntialized() const { return !submap_collection_ptr_->empty(); }
  void intializeMap(const Transformation& T_G_C);

  // Submap creation
  bool newSubmapRequired() const;
  void createNewSubmap(const Transformation& T_G_C, const ros::Time& timestamp);
  inline void finishSubmap(const SubmapID& submap_id);

  // Submap publishing
  void publishPose(SubmapID submap_id) const;
  void PoseCallback(const cblox_msgs::MapPoseUpdate& msg);
  void publishSubmap(SubmapID submap_id, bool global_map = false) const;
  void SubmapCallback(const cblox_msgs::MapLayerPtr& msg);
  void publishWholeMap() const;

  // visualization
  void visualizeSlice(const SubmapID& submap_id) const;
  float slice_height_ = 1.5;
  ros::Publisher sdf_slice_pub_;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber submap_sub_;
  ros::Subscriber pose_sub_;

  // Publishers
  ros::Publisher active_submap_mesh_pub_;
  ros::Publisher submap_poses_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher submap_pub_;
  ros::Publisher pose_pub_;

  // Services
  ros::ServiceServer generate_separated_mesh_srv_;
  ros::ServiceServer generate_combined_mesh_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;

  // Timers.
  ros::Timer update_mesh_timer_;

  bool verbose_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  std::string world_frame_;

  // The submap collection
  std::shared_ptr<SubmapCollection<SubmapType>> submap_collection_ptr_;

  // The integrator
  std::shared_ptr<TsdfSubmapCollectionIntegrator>
      tsdf_submap_collection_integrator_ptr_;

  // For meshing the entire collection to file
  std::shared_ptr<SubmapMesher> submap_mesher_ptr_;
  std::string mesh_filename_;

  // For meshing the active layer
  std::shared_ptr<ActiveSubmapVisualizer> active_submap_visualizer_ptr_;

  // For visualizing the trajectory
  std::shared_ptr<TrajectoryVisualizer> trajectory_visualizer_ptr_;

  // Transformer object to keep track of either TF transforms or messages from a
  // transform topic.
  voxblox::Transformer transformer_;

  // The queue of unprocessed pointclouds
  std::queue<sensor_msgs::PointCloud2::Ptr> pointcloud_queue_;
  std::queue<cblox_msgs::MapLayerPtr> submap_queue_;

  // Last message times for throttling input.
  ros::Duration min_time_between_msgs_;
  ros::Time last_msg_time_ptcloud_;

  /// Colormap to use for intensity pointclouds.
  std::unique_ptr<voxblox::ColorMap> color_map_;

  // Number of frames integrated to the current submap
  int num_integrated_frames_current_submap_;
  // The number of frames integrated into a submap before requesting a new one.
  int num_integrated_frames_per_submap_;

  mutable std::mutex visualizer_mutex;
};

}  // namespace cblox

#include "cblox_ros/submap_server_inl.h"

#endif  // CBLOX_ROS_SUBMAP_SERVER__H_
