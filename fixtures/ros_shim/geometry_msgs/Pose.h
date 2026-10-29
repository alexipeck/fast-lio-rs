#pragma once
#include <array>
#include <ros/ros.h>

namespace geometry_msgs {
struct Point { double x = 0, y = 0, z = 0; };
struct Quaternion { double x = 0, y = 0, z = 0, w = 1; };
struct Pose { Point position; Quaternion orientation; };
struct PoseStamped { std_msgs::Header header; Pose pose; };
struct PoseWithCovariance { Pose pose; std::array<double, 36> covariance{}; };
}  // namespace geometry_msgs
