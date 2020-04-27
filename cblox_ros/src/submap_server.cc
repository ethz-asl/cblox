#include "cblox_ros/submap_server.h"

namespace cblox {

template <>
void SubmapServer<TsdfSubmap>::visualizeSlice(const SubmapID submap_id) const {
  if (sdf_slice_pub_.getNumSubscribers() < 1) {
    return;
  }
  if (!submap_collection_ptr_->exists(submap_id)) {
    ROS_WARN("[CbloxServer] Submap %d does not exist.", submap_id);
    return;
  }

  if (verbose_) {
    ROS_INFO("[CbloxServer] Visualizing TSDF slice of submap %d at height %.2f",
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

  const float max_dist = truncation_distance_;
  TsdfSubmap::Ptr submap_ptr = submap_collection_ptr_->getSubmapPtr(submap_id);
  voxblox::Layer<voxblox::TsdfVoxel>* layer =
      submap_ptr->getTsdfMapPtr()->getTsdfLayerPtr();
  voxblox::BlockIndexList block_list;
  layer->getAllAllocatedBlocks(&block_list);
  int block_num = 0;
  for (const voxblox::BlockIndex& block_id : block_list) {
    if (!layer->hasBlock(block_id)) continue;
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
        layer->getBlockPtrByIndex(block_id);
    for (size_t voxel_id = 0; voxel_id < block->num_voxels(); voxel_id++) {
      const voxblox::TsdfVoxel& voxel = block->getVoxelByLinearIndex(voxel_id);
      voxblox::Point position =
          submap_ptr->getPose() *
          block->computeCoordinatesFromLinearIndex(voxel_id);

      if (voxel.weight < 1e-6) {
        continue;
      }

      color_msg.r = 0.0;
      color_msg.g = 0.0;
      if (voxel.weight >= 1e-6) {
        color_msg.r = std::max(
            std::min((max_dist - voxel.distance) / 2.0 / max_dist, 1.0), 0.0);
        color_msg.g = std::max(
            std::min((max_dist + voxel.distance) / 2.0 / max_dist, 1.0), 0.0);
      }

      if (std::abs(position.z() - slice_height_) <
          submap_ptr->getTsdfMapPtr()->voxel_size() / 2) {
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

}  // namespace cblox
