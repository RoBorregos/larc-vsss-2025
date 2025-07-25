#pragma once
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace tf2;

void VectorFromMSG( geometry_msgs::msg::Vector3& r, Vector3& o);

void QuaternionFromMSG( geometry_msgs::msg::Quaternion& r, Quaternion& o);

void TransformFromMSG( geometry_msgs::msg::Transform& r, Transform& o);

void SetTransformFromOdom(const nav_msgs::msg::Odometry::SharedPtr& msg, Transform& o);

void MSGFromVector3(Vector3& r,  geometry_msgs::msg::Vector3& o);