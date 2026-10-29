#pragma once
// Minimal stand-in for <ros/ros.h>: time, header, logging, and inert node,
// publisher, subscriber and rate types so laserMapping.cpp compiles.
#include <cassert>
#include <cstdio>
#include <iomanip>  // real ROS headers bring it in; laserMapping.cpp uses setw
#include <iostream>
#include <string>

namespace ros {
struct Time {
  double sec_ = 0.0;
  double toSec() const { return sec_; }
  Time &fromSec(double s) { sec_ = s; return *this; }
  static Time now() { return Time(); }
};
struct Publisher {
  template <class M> void publish(const M &) const {}
};
struct Subscriber {};
struct NodeHandle {
  template <class T, class D> bool param(const std::string &, T &v, const D &d) { v = d; return true; }
  template <class... A> Subscriber subscribe(A &&...) { return {}; }
  template <class M> Publisher advertise(const std::string &, int) { return {}; }
};
struct Rate {
  explicit Rate(double) {}
  void sleep() {}
};
inline void init(int &, char **, const std::string &) {}
inline void spinOnce() {}
inline bool ok() { return true; }
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
