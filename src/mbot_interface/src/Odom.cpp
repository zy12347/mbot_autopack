#include "mbot_interface/Odom.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

void Odom::FromRosMsg(const nav_msgs::msg::Odometry::SharedPtr& ros_odom) {
  // 位置信息
  x = ros_odom->pose.pose.position.x;
  y = ros_odom->pose.pose.position.y;
  z = ros_odom->pose.pose.position.z;

  // 从四元数转换为欧拉角
  tf2::Quaternion q(
      ros_odom->pose.pose.orientation.x, ros_odom->pose.pose.orientation.y,
      ros_odom->pose.pose.orientation.z, ros_odom->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  // 速度信息
  linear_x = ros_odom->twist.twist.linear.x;
  linear_y = ros_odom->twist.twist.linear.y;
  angular_z = ros_odom->twist.twist.angular.z;

  // 时间戳
  stamp = ros_odom->header.stamp.sec * 1000000000ULL +
          ros_odom->header.stamp.nanosec;
}

nav_msgs::msg::Odometry Odom::ToRosMsg() const {
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_footprint";

  // 位置信息
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = z;

  // 从欧拉角转换为四元数
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // 速度信息
  msg.twist.twist.linear.x = linear_x;
  msg.twist.twist.linear.y = linear_y;
  msg.twist.twist.angular.z = angular_z;

  // 时间戳
  msg.header.stamp.sec = stamp / 1000000000ULL;
  msg.header.stamp.nanosec = stamp % 1000000000ULL;

  return msg;
}