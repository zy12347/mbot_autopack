#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <map>
#include <chrono>
#include <cstdio>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // 雷达消息类型
#include "mbot_interface/msg/person.hpp"
#include "mbot_interface/msg/grid_map.hpp"
#include "mbot_interface/msg/pose2_d.hpp"

#include "rclcpp/rclcpp.hpp"


#include "Pose2D.h"
#include "LaserScan.h"
#include "Odom.h"
#include "GridMap.h"