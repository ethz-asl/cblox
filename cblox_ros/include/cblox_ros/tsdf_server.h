#ifndef CBLOX_ROS_TSDF_SERVER_H_
#define CBLOX_ROS_TSDF_SERVER_H_

#include <memory>
#include <string>
#include <vector>

//#include <Eigen/Geometry>

//#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <std_srvs/Empty.h>

#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>

#include <voxblox_ros/transformer.h>

//#include <pcl/conversions.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>

//#include <message_filters/subscriber.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/time_synchronizer.h>

//#include <kindr/minimal/quat-transformation.h>

//#include <cblox/core/tsdf_submap_collection.h>
//#include <cblox/mesh/tsdf_submap_mesher.h>

//#include "manifold_mapping/core/common.hpp"
//#include "manifold_mapping/core/frame_integrator.hpp"
//#include "manifold_mapping/core/manifold_maintainer.hpp"
//#include "manifold_mapping/core/sparse/sparse_mapper.hpp"
//#include "manifold_mapping/core/sparse/trackable_frame.hpp"
//#include "manifold_mapping/core/transformer.hpp"

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/core/tsdf_submap.h>
#include <cblox/integrator/tsdf_submap_collection_integrator.h>

namespace cblox {

// Default values for parameters
constexpr bool kDefaultVerbose = true;
constexpr size_t kDefaultNumFramesPerSubmap = 20;
constexpr size_t kDefaultNumKeyFramesPerSubmap = 10;
constexpr double kDefaultMinTimeBetweenMsgsSec = 0.0;

// Data queue sizes
constexpr int kDefaultPointcloudQueueSize = 1;
// constexpr int kDefaultFrameQueueSize = 1;
// constexpr int kDefaultStereoImageSynchronizerQueueSize = 10;
// constexpr int kDefaultRGBDImageSynchronizerQueueSize = 10;

// The synchronization policy used by the interface to sync stereo images
// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
//                                                        sensor_msgs::Image>
//    sync_pol;

// Class handling global alignment calculation and publishing
class TsdfServer {
 public:
  // Constructor
  TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
             const TsdfMap::Config& tsdf_map_config,
             const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
             const voxblox::TsdfIntegratorType& tsdf_integrator_type,
             const voxblox::MeshIntegratorConfig& mesh_config);
  virtual ~TsdfServer() {}

  // Pointcloud data subscriber
  virtual void pointcloudCallback(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);

  // Saving and Loading callbacks
  // bool saveMapCallback(std_srvs::Empty::Request& request,     // NOLINT
  //                     std_srvs::Empty::Response& response);  // NOLINT
  // bool loadMapCallback(std_srvs::Empty::Request& request,     // NOLINT
  //                     std_srvs::Empty::Response& response);  // NOLINT

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
  void createNewSubMap(const Transformation& T_G_C);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool verbose_;

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string world_frame_;

  // Pointcloud subscriber
  ros::Subscriber pointcloud_sub_;

  // The submap collection
  std::shared_ptr<SubmapCollection<TsdfSubmap>> tsdf_submap_collection_ptr_;

  // The integrator
  std::shared_ptr<TsdfSubmapCollectionIntegrator>
      tsdf_submap_collection_integrator_ptr_;

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

};

}  // namespace cblox

#endif /* CBLOX_ROS_TSDF_SERVER_H_ */
