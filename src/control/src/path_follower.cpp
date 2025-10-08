#include "path_follower.h"

void PathFollower::setAngularParams(float angular_kp, float angular_ki,
                                    float angular_kd, float min_angular,
                                    float max_angular) {
  if (angular_pid_ == nullptr) {
    angular_pid_ = std::make_unique<PIDController>(
        angular_kp, angular_ki, angular_kd, min_angular, max_angular);
  } else {
    angular_pid_->setParams(angular_kp, angular_ki, angular_kd, min_angular,
                            max_angular);
  }
}

void PathFollower::setLinearParams(float linear_kp, float linear_ki,
                                   float linear_kd, float min_linear,
                                   float max_linear) {
  if (linear_pid_ == nullptr) {
    linear_pid_ = std::make_unique<PIDController>(
        linear_kp, linear_ki, linear_kd, min_linear, max_linear);
  } else {
    linear_pid_->setParams(linear_kp, linear_ki, linear_kd, min_linear,
                           max_linear);
  }
}

void PathFollower::run(const geometry_msgs::msg::PoseStamped& current_pose,
                       float& v, float& w) {
  auto target_waypoint_opt = findTargetWayPoint(current_pose);
  if (!target_waypoint_opt.has_value()) {
    // 无有效路点：停止运动（或其他错误处理）
    v = 0.0f;
    w = 0.0f;
    RCLCPP_WARN(rclcpp::get_logger("PathFollower"), "未找到目标路点，停止运动");
    return;
  }
  geometry_msgs::msg::PoseStamped target_waypoint = target_waypoint_opt.value();
  float dis_error = std::hypot(
      target_waypoint.pose.position.x - current_pose.pose.position.x,
      target_waypoint.pose.position.y - current_pose.pose.position.y);
  float target_v = linear_pid_->compute(dis_error);
  //   std::cout << " pose: delta_dis: target_v: " <<
  //   target_waypoint.pose.position.x
  //             << " " << current_pose.pose.position.x << " "
  //             << target_waypoint.pose.position.y << " "
  //             << current_pose.pose.position.y << " " << dis_error << ""
  //             << target_v << std::endl;
  float target_pose_angle = std::atan2(
      target_waypoint.pose.position.y - current_pose.pose.position.y,
      target_waypoint.pose.position.x - current_pose.pose.position.x);
  float cur_pose_angle = 2 * std::atan2(current_pose.pose.orientation.z,
                                        current_pose.pose.orientation.w);
  float delta_error = Pose2D::Wrap2PI(target_pose_angle - cur_pose_angle);
  float target_w = angular_pid_->compute(delta_error);
  //   std::cout << " angle: delta_angle: target_v: " << " " <<
  //   target_pose_angle
  //             << " " << cur_pose_angle << " " << delta_error << " " <<
  //             target_w
  //             << std::endl;
  v = target_v;
  w = target_w;
}

std::optional<geometry_msgs::msg::PoseStamped> PathFollower::findTargetWayPoint(
    const geometry_msgs::msg::PoseStamped& current_pose) {
  double lookahead = 0.1;
  size_t num_waypoints = current_path_.poses.size();
  if (num_waypoints == 0) {
    RCLCPP_WARN(rclcpp::get_logger("PathFollower"),
                "当前路径为空，无法查找目标路点");
    return std::nullopt;
  }
  // 从当前索引开始查找第一个距离大于前瞻距离的路点
  for (size_t i = cur_waypoint_index_; i < num_waypoints; ++i) {
    double dist = calculateDistance(current_pose, current_path_.poses[i]);
    if (dist > lookahead || i == num_waypoints - 1) {
      cur_waypoint_index_ = i; // 更新当前索引
      return current_path_.poses[i];
    }
  }
  return std::nullopt;
}