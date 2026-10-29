#pragma once
// Minimal stand-in for <ros/ros.h>: time, header, publisher placeholder and logging.
#include <cassert>
#include <cstdio>
#include <string>

namespace ros {
struct Time {
  double sec_ = 0.0;
  double toSec() const { return sec_; }
  Time &fromSec(double s) { sec_ = s; return *this; }
  static Time now() { return Time(); }
};
struct Publisher {};
inline int &info_count() { static int n = 0; return n; }
}  // namespace ros

namespace std_msgs {
struct Header {
  ros::Time stamp;
  std::string frame_id;
};
}  // namespace std_msgs

#define ROS_INFO(...) (++ros::info_count(), std::fprintf(stderr, __VA_ARGS__), std::fputc('\n', stderr))
#define ROS_WARN(...) (std::fprintf(stderr, __VA_ARGS__), std::fputc('\n', stderr))
#define ROS_ERROR(...) (std::fprintf(stderr, __VA_ARGS__), std::fputc('\n', stderr))
#define ROS_ASSERT(cond) assert(cond)
