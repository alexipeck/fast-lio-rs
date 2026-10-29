#pragma once
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace sensor_msgs {
struct PointCloud2 {
  std_msgs::Header header;
  typedef boost::shared_ptr<const PointCloud2> ConstPtr;
};
}  // namespace sensor_msgs
