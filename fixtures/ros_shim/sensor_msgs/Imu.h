#pragma once
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

namespace sensor_msgs {
struct Imu {
  std_msgs::Header header;
  geometry_msgs::Vector3 angular_velocity;
  geometry_msgs::Vector3 linear_acceleration;
  typedef boost::shared_ptr<Imu> Ptr;
  typedef boost::shared_ptr<const Imu> ConstPtr;
};
typedef Imu::Ptr ImuPtr;
typedef Imu::ConstPtr ImuConstPtr;
}  // namespace sensor_msgs
