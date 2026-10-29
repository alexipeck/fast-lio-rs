#pragma once
#include <string>
#include <geometry_msgs/Pose.h>
namespace nav_msgs {
struct Odometry {
  std_msgs::Header header;
  std::string child_frame_id;
  geometry_msgs::PoseWithCovariance pose;
};
}  // namespace nav_msgs
