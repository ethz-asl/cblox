#ifndef CBLOX_ROS_TSDF_SERVER_H_
#define CBLOX_ROS_TSDF_SERVER_H_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <voxblox/utils/color_maps.h>
#include <voxblox_ros/transformer.h>

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/core/tsdf_submap.h>
#include <cblox/integrator/tsdf_submap_collection_integrator.h>
#include <cblox/mesh/submap_mesher.h>

#include "cblox_ros/active_submap_visualizer.h"
#include "cblox_ros/trajectory_visualizer.h"

namespace cblox {

// Default values for parameters
constexpr bool kDefaultVerbose = true;
constexpr size_t kDefaultNumFramesPerSubmap = 20;
constexpr double kDefaultMinTimeBetweenMsgsSec = 0.0;

// Data queue sizes
constexpr int kDefaultPointcloudQueueSize = 1;

// Receives ROS Data and produces a collection of submaps
class TsdfSubmapServer {
 public:
  // Constructor
  TsdfSubmapServer(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  TsdfSubmapServer(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      const TsdfMap::Config& tsdf_map_config,
      const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
      const voxblox::TsdfIntegratorType& tsdf_integrator_type,
      const voxblox::MeshIntegratorConfig& mesh_config);
  virtual ~TsdfSubmapServer() {}

  // Pointcloud data subscriber
  virtual void pointcloudCallback(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);

  // Saving and Loading callbacks
  // bool saveMapCallback(std_srvs::Empty::Request& request,     // NOLINT
  //                     std_srvs::Empty::Response& response);  // NOLINT
  // bool loadMapCallback(std_srvs::Empty::Request& request,     // NOLINT
  //                     std_srvs::Empty::Response& response);  // NOLINT

  // Mesh output
  bool generateSeparatedMeshCallback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);
  bool generateCombinedMeshCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);

 private:
  // Gets parameters
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

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
  bool mapIntialized() const { return !tsdf_submap_collection_ptr_->empty(); }
  void intializeMap(const Transformation& T_G_C);

  // Submap creation
  bool newSubmapRequired() const;
  void createNewSubMap(const Transformation& T_G_C);

  // Update the mesh and publish for visualization
  void updateMeshEvent(const ros::TimerEvent& /*event*/);
  void updateActiveSubmapMesh();

  // Visualize submap base frames
  void visualizeSubMapBaseframes() const;

  // Visualize the trajectory
  void visualizeTrajectory() const;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber pointcloud_sub_;

  // Publishers
  ros::Publisher active_submap_mesh_pub_;
  ros::Publisher submap_poses_pub_;
  ros::Publisher trajectory_pub_;

  // Services
  ros::ServiceServer generate_separated_mesh_srv_;
  ros::ServiceServer generate_combined_mesh_srv_;

  // Timers.
  ros::Timer update_mesh_timer_;

  bool verbose_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  std::string world_frame_;

  // The submap collection
  std::shared_ptr<SubmapCollection<TsdfSubmap>> tsdf_submap_collection_ptr_;

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

  // Last message times for throttling input.
  ros::Duration min_time_between_msgs_;
  ros::Time last_msg_time_ptcloud_;

  /// Colormap to use for intensity pointclouds.
  std::unique_ptr<voxblox::ColorMap> color_map_;

  // Number of frames integrated to the current submap
  size_t num_integrated_frames_current_submap_;
  // The number of frames integrated into a submap before requesting a new one.
  size_t num_integrated_frames_per_submap_;
};

}  // namespace cblox

#endif /* CBLOX_ROS_TSDF_SERVER_H_ */
