#pragma once
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace livox_ros_driver {
struct CustomMsg {
  std_msgs::Header header;
  typedef boost::shared_ptr<const CustomMsg> ConstPtr;
};
}  // namespace livox_ros_driver
