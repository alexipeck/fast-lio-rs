#pragma once
#include <vector>
#include <geometry_msgs/Pose.h>
namespace nav_msgs {
struct Path {
  std_msgs::Header header;
  std::vector<geometry_msgs::PoseStamped> poses;
};
}  // namespace nav_msgs
