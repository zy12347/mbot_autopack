#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp" // Path 消息头文件
#include "path_follower.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MotionController : public rclcpp::Node {
 public:
  MotionController() : Node("mbot_motion_controller") {
    // twist.linear.x = 0.2;
    // twist.angular.z = 0.5;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/mbot/cmd_vel", 10);
    // subscriber_ =
    // this->create_subscription<mbot_interface::msg::Person>("mbot_topic",
    // 10,[this](const mbot_interface::msg::Person::SharedPtr
    // msg){this->event_callback(msg);});
    gazebo_pose_sub_ptr_ =
        this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            [this](const gazebo_msgs::msg::ModelStates& msg) {
              this->gazebo_pose_event_callback(msg);
            });
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                     [this]() { this->timer_callback(); });

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10, [this](const nav_msgs::msg::Path& msg) {
          this->path_event_callback(msg);
        });
  }

 private:
  geometry_msgs::msg::Twist twist;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr
      gazebo_pose_sub_ptr_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  //  rclcpp::Subscription<mbot_interface::msg::Person>::SharedPtr subscriber_;
  void gazebo_pose_event_callback(const gazebo_msgs::msg::ModelStates& msg) {
    int id = -1;
    for (int i = 0; i < msg.name.size(); i++) {
      if (msg.name[i] == "mbot") {
        id = i;
        break;
      }
    }
    if (id == -1) {
      RCLCPP_WARN(this->get_logger(), "未找到模型'mbot'");
      return;
    }
    cur_pose_.header.frame_id = "world";
    cur_pose_.header.stamp = this->get_clock()->now();
    cur_pose_.pose = msg.pose[id];
  }

  void path_event_callback(const nav_msgs::msg::Path& msg) {
    if (msg.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "无路径");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Start Following Path");
    if (pid_path_follower_ == nullptr) {
      pid_path_follower_ = std::make_unique<PathFollower>();
      pid_path_follower_->setLinearParams(0.1, 0.01, 0, -0.3, 0.3, 0.1);
      pid_path_follower_->setAngularParams(0.1, 0.01, 0, -3.14, 3.14, 0.75);
    }
    pid_path_follower_->setPath(msg);
    path_updated_ = true;
  }

  // 定时器回调：循环执行路径跟踪并发布速度
  void timer_callback() {
    // 若未收到路径，发布零速度并退出
    if (!path_updated_ || pid_path_follower_ == nullptr) {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      cmd_publisher_->publish(twist);
      return;
    }

    // 线程安全地读取当前位姿
    geometry_msgs::msg::PoseStamped current_pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      current_pose = cur_pose_;
    }
    if (current_pose.header.stamp.sec == 0) { // 未收到过Gazebo位姿时，时间戳为0
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "未收到机器人位姿，等待Gazebo数据...");
      return;
    }

    // 循环执行路径跟随计算（核心修改：在定时器中持续调用）
    // 假设run方法返回计算后的速度指令（需修改PIDPathFollower实现）
    float target_v = 0;
    float target_w = 0;
    pid_path_follower_->run(current_pose, target_v, target_w);
    twist.linear.x = target_v;
    twist.angular.z = target_w;
    if (pid_path_follower_->isReached()) {
      RCLCPP_INFO(this->get_logger(), "Arrived");
      path_updated_ = false;
      return;
    }
    // 发布速度
    cmd_publisher_->publish(twist);
  }

  geometry_msgs::msg::PoseStamped cur_pose_;
  std::unique_ptr<PathFollower> pid_path_follower_;
  bool path_updated_ = false;
  std::mutex pose_mutex_;
  // void event_callback(const mbot_interface::msg::Person::SharedPtr msg){
  //   RCLCPP_INFO(this->get_logger(), "收到消息: '%s'", msg->name.c_str());
  // }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}