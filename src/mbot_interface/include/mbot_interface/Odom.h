#pragma once
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"

class Odom {
public:
    // 位置信息
    double x = 0.0;     // x坐标
    double y = 0.0;     // y坐标
    double z = 0.0;     // z坐标（通常用于3D，但地面机器人可能为0）
    
    // 姿态信息（欧拉角）
    double roll = 0.0;  // 横滚角
    double pitch = 0.0; // 俯仰角
    double yaw = 0.0;   // 偏航角（最重要，机器人朝向）
    
    // 速度信息
    double linear_x = 0.0;  // x方向线速度
    double linear_y = 0.0;  // y方向线速度
    double angular_z = 0.0; // z方向角速度（转向速度）

    // 时间戳
    uint64_t stamp;

    // 从ROS Odometry消息转换为自定义Odom类
    void FromRosMsg(const nav_msgs::msg::Odometry::SharedPtr& ros_odom);
    
    // 转换为ROS Odometry消息
    nav_msgs::msg::Odometry ToRosMsg() const;
};