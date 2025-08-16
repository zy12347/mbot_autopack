#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "mbot_interface/GridMap.h"
#include "mbot_interface/KDTree.h"
#include "mbot_interface/LaserScan.h"
#include "mbot_interface/Odom.h"
#include "mbot_interface/Pose2D.h"
// #include "mbot_interface/msg/grid_map.hpp"
#include "mbot_interface/msg/person.hpp"
#include "mbot_interface/msg/pose2_d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // 雷达消息类型