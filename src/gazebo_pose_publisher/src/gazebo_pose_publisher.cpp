#include <chrono>
#include <memory>
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class GazeboPosePublisher : public rclcpp::Node {
 public:
  GazeboPosePublisher() : Node("gazebo_pose_publisher") {
    // 声明并获取参数
    this->declare_parameter<std::string>("robot_name", "mbot");
    this->declare_parameter<std::string>("publish_topic", "/mbot/gazebo_pose");
    this->declare_parameter<double>("update_rate", 30.0);
    this->declare_parameter<std::string>("frame_id", "world");

    this->get_parameter("robot_name", robot_name_);
    this->get_parameter("publish_topic", publish_topic_);
    this->get_parameter("update_rate", update_rate_);
    this->get_parameter("frame_id", frame_id_);

    // 创建服务客户端
    client_ = this->create_client<gazebo_msgs::srv::GetEntityState>(
        "/gazebo/get_entity_state");

    // 创建发布者
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        publish_topic_, 10);

    // 创建定时器，按指定频率调用回调函数
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / update_rate_),
        std::bind(&GazeboPosePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Gazebo pose publisher initialized");
    RCLCPP_INFO(this->get_logger(), "Robot name: %s", robot_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s",
                publish_topic_.c_str());
  }

 private:
  void timer_callback() {
    // 检查服务是否可用
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for /gazebo/get_entity_state service...");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Sending request: name=%s, reference_frame=%s",
                robot_name_.c_str(), frame_id_.c_str()); // 关键日志
    // 创建服务请求
    auto request =
        std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = robot_name_;
    request->reference_frame = frame_id_;

    // 发送异步请求
    auto result_future = client_->async_send_request(
        request, std::bind(&GazeboPosePublisher::response_callback, this,
                           std::placeholders::_1));
  }

  // 服务响应回调函数
  void response_callback(
      rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future) {
    try {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(),
                  "Service response: success=%d, message=%s", response->success,
                  response->status_message.c_str()); // 关键日志
      if (response->success) {
        // 构造并发布位姿消息
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose = response->state.pose;
        publisher_->publish(pose_msg);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pose for robot: %s",
                     robot_name_.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
  }

  // 成员变量
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_name_;
  std::string publish_topic_;
  double update_rate_;
  std::string frame_id_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
