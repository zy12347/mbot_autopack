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