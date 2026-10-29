#pragma once
// The real header pulls in PCL's point-type machinery, which preprocess.h
// relies on. The ROS message conversions are inert.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <sensor_msgs/PointCloud2.h>
namespace pcl {
template <class T> void toROSMsg(const pcl::PointCloud<T> &, sensor_msgs::PointCloud2 &) {}
template <class T> void fromROSMsg(const sensor_msgs::PointCloud2 &, pcl::PointCloud<T> &) {}
}  // namespace pcl
