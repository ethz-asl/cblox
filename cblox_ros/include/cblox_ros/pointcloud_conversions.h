#ifndef CBLOX_ROS_POINTCLOUD_CONVERSIONS_H_
#define CBLOX_ROS_POINTCLOUD_CONVERSIONS_H_

#include <sensor_msgs/PointCloud2.h>

#include <cblox/core/common.h>
#include <voxblox/utils/color_maps.h>

namespace cblox {

// Convert the PCL pointcloud into our awesome format.
inline void convertPointcloudMsg(
    const voxblox::ColorMap& color_map,
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    Pointcloud* points_C_ptr, Colors* colors_ptr) {
  CHECK(pointcloud_msg);
  CHECK_NOTNULL(points_C_ptr);
  CHECK_NOTNULL(colors_ptr);

  // Horrible hack fix to fix color parsing colors_ptr->in PCL.
  bool color_pointcloud = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      color_pointcloud = true;
    }
  }

  timing::Timer ptcloud_timer("ptcloud_preprocess");

  // Convert differently depending on RGB or I type.
  if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    points_C_ptr->reserve(pointcloud_pcl.size());
    colors_ptr->reserve(pointcloud_pcl.size());
    for (size_t i = 0; i < pointcloud_pcl.points.size(); ++i) {
      if (!std::isfinite(pointcloud_pcl.points[i].x) ||
          !std::isfinite(pointcloud_pcl.points[i].y) ||
          !std::isfinite(pointcloud_pcl.points[i].z)) {
        continue;
      }
      points_C_ptr->push_back(Point(pointcloud_pcl.points[i].x,
                                    pointcloud_pcl.points[i].y,
                                    pointcloud_pcl.points[i].z));
      colors_ptr->push_back(
          Color(pointcloud_pcl.points[i].r, pointcloud_pcl.points[i].g,
                pointcloud_pcl.points[i].b, pointcloud_pcl.points[i].a));
    }
  } else {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    points_C_ptr->reserve(pointcloud_pcl.size());
    colors_ptr->reserve(pointcloud_pcl.size());
    for (size_t i = 0; i < pointcloud_pcl.points.size(); ++i) {
      if (!std::isfinite(pointcloud_pcl.points[i].x) ||
          !std::isfinite(pointcloud_pcl.points[i].y) ||
          !std::isfinite(pointcloud_pcl.points[i].z)) {
        continue;
      }
      points_C_ptr->push_back(Point(pointcloud_pcl.points[i].x,
                                    pointcloud_pcl.points[i].y,
                                    pointcloud_pcl.points[i].z));
      colors_ptr->push_back(
          color_map.colorLookup(pointcloud_pcl.points[i].intensity));
    }
  }
  ptcloud_timer.Stop();
}

}  // namespace cblox

#endif  // CBLOX_ROS_POINTCLOUD_CONVERSIONS_H_
