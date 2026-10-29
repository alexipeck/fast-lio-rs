#pragma once
// Inert TF types: FAST-LIO broadcasts its pose; the fixtures don't need it.
#include <string>
#include <ros/ros.h>
namespace tf {
struct Vector3 { Vector3(double, double, double) {} };
struct Quaternion { void setW(double) {} void setX(double) {} void setY(double) {} void setZ(double) {} };
struct Transform { void setOrigin(const Vector3 &) {} void setRotation(const Quaternion &) {} };
struct StampedTransform { StampedTransform(const Transform &, const ros::Time &, const std::string &, const std::string &) {} };
struct TransformBroadcaster { void sendTransform(const StampedTransform &) {} };
}  // namespace tf
