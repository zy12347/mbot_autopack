#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mbot_interface/mbot_common.h"
#include "nav_msgs/msg/path.hpp" // Path 消息头文件
#include "pid_controller.h"
class PathFollower {
 public:
  PathFollower() {};
  void setLinearParams(float linear_kp, float linear_ki, float linear_kd,
                       float min_linear, float max_linear);

  void setAngularParams(float angular_kp, float angular_ki, float angular_kd,
                        float min_angular, float max_angular);
  void setPath(const nav_msgs::msg::Path& msg) {
    cur_waypoint_index_ = 0;
    current_path_ = msg;
  }
  bool isReached() {
    return cur_waypoint_index_ == current_path_.poses.size() - 1;
  }
  void run(const geometry_msgs::msg::PoseStamped& current_pose, float& v,
           float& w);

 private:
  std::optional<geometry_msgs::msg::PoseStamped> findTargetWayPoint(
      const geometry_msgs::msg::PoseStamped& current_pose);

  double calculateDistance(const geometry_msgs::msg::PoseStamped& a,
                           const geometry_msgs::msg::PoseStamped& b) {
    if (a.header.frame_id != b.header.frame_id) {
      RCLCPP_ERROR(rclcpp::get_logger("PathFollower"),
                   "当前位姿坐标系(%s)与路径坐标系(%s)不一致，无法计算距离",
                   a.header.frame_id.c_str(), b.header.frame_id.c_str());
      return -1;
    }
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }
  std::unique_ptr<PIDController> linear_pid_;
  std::unique_ptr<PIDController> angular_pid_;
  int cur_waypoint_index_ = 0;
  nav_msgs::msg::Path current_path_;
};