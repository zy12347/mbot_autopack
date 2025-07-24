#include <chrono>
#include <cstdio>
#include <memory>

#include "mbot_interface/msg/person.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MbotManager : public rclcpp::Node {
 public:
  MbotManager() : Node("mbot_manager") {
    publisher_ =
        this->create_publisher<mbot_interface::msg::Person>("mbot_topic", 10);
    subscriber_ = this->create_subscription<mbot_interface::msg::Person>(
        "mbot_topic", 10,
        [this](const mbot_interface::msg::Person::SharedPtr msg) {
          this->event_callback(msg);
        });
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     [this]() { this->timer_callback(); });
    person.name = "John";
    person.age = 20;
    person.height = 1.75;
  }

 private:
  mbot_interface::msg::Person person;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mbot_interface::msg::Person>::SharedPtr publisher_;
  rclcpp::Subscription<mbot_interface::msg::Person>::SharedPtr subscriber_;

  void timer_callback() {
    // 发布数据
    publisher_->publish(person);
  }
  void event_callback(const mbot_interface::msg::Person::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "收到消息: '%s'", msg->name.c_str());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MbotManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
