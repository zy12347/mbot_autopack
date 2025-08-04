#include "mbot_interface/LaserScan.h"

void LaserScan::FromRosMsg(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  stamp = msg->header.stamp;
  frame_id = msg->header.frame_id;
  angle_min = msg->angle_min;
  angle_max = msg->angle_max;
  angle_increment = msg->angle_increment;
  range_min = msg->range_min;
  range_max = msg->range_max;
  ranges = msg->ranges;
  intensities = msg->intensities;

  // 添加调试信息
  std::cout << "LaserScan::FromRosMsg: ranges.size()=" << ranges.size()
            << ", range_min=" << range_min << ", range_max=" << range_max
            << ", angle_min=" << angle_min << ", angle_max=" << angle_max
            << ", angle_increment=" << angle_increment << std::endl;

  // 检查有效距离的数量
  int valid_ranges = 0;
  for (const auto& range : ranges) {
    if (range >= range_min && range <= range_max) {
      valid_ranges++;
    }
  }
  std::cout << "LaserScan::FromRosMsg: valid_ranges=" << valid_ranges << "/"
            << ranges.size() << std::endl;
}

sensor_msgs::msg::LaserScan LaserScan::ToRosMsg() const {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.angle_min = angle_min;
  msg.angle_max = angle_max;
  msg.angle_increment = angle_increment;
  msg.range_min = range_min;
  msg.range_max = range_max;
  msg.ranges = ranges;
  msg.intensities = intensities;
  return msg;
}