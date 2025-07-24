#include <chrono>
#include <cstdio>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MotionController : public rclcpp::Node {
 public:
  MotionController() : Node("mbot_motion_controller") {
    twist.linear.x = 0.2;
    twist.angular.z = 0.5;
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/mbot/cmd_vel", 10);
    // subscriber_ =
    // this->create_subscription<mbot_interface::msg::Person>("mbot_topic",
    // 10,[this](const mbot_interface::msg::Person::SharedPtr
    // msg){this->event_callback(msg);});
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     [this]() { this->timer_callback(); });
  }

 private:
  geometry_msgs::msg::Twist twist;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  //  rclcpp::Subscription<mbot_interface::msg::Person>::SharedPtr subscriber_;

  void timer_callback() {
    // 发布数据
    publisher_->publish(twist);
  }
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