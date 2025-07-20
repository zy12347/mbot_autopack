#include <cstdio>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Publisher: public rclcpp::Node{
    public:
        Publisher();
        ~Publisher();

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std::chrono::seconds period_;
        std::string topic_name_;
        std::string message_;
        
}