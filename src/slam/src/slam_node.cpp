#include "gmapping/gmapping.h"

class SlamManager : public rclcpp::Node {
 public:
  SlamManager() : Node("slam") {
    // publisher_ = this->create_publisher<mbot_interface::msg::Person>("", 10);
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "mbot/laser_plugin/out", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->laser_event_callback(msg);
        });
    // timer_ =
    // this->create_wall_timer(std::chrono::seconds(1),[this](){this->timer_callback();});
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "mbot/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->odom_event_callback(msg);
        });

    // auto period = std::chrono::duration<double>(1.0 / freq);
    // main_loop_timer_ = this->create_wall_timer(
    //     period,
    //     std::bind(&SlamManager::main_loop, this)
    // );

    // 初始化发布者
    map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    pose_publisher_ =
        this->create_publisher<mbot_interface::msg::Pose2D>("robot_pose", 10);

    RCLCPP_INFO(this->get_logger(), "SLAM节点初始化完成,主循环频率: %.1f Hz",
                freq);

    // 初始化Gmapping，设置初始位姿
    RCLCPP_INFO(this->get_logger(), "Gmapping初始化");
    Pose2D init_pose(0.0, 0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Gmapping初始化");
    gmapping_.Initialize(init_pose);
  }

 private:
  //  mbot_interface::msg::Person person;
  //  rclcpp::TimerBase::SharedPtr timer_;
  //  rclcpp::Publisher<mbot_interface::msg::Person>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  rclcpp::Publisher<mbot_interface::msg::Pose2D>::SharedPtr pose_publisher_;

  void laser_event_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!msg) {
      RCLCPP_WARN(this->get_logger(), "收到空的激光扫描消息");
      return;
    }

    try {
      laser_scan_.FromRosMsg(msg);

      gmapping_.ProcessScan(laser_scan_, odo_);

      if (pose_publisher_) {
        pose_publisher_->publish(gmapping_.GetBestEstimate().ToRosMsg());
      }

      // RCLCPP_INFO(rclcpp::get_logger("gazebo_subscriber"), "grid_map");
      GridMap& grid = gmapping_.GetGridMap();
      // grid.SaveAsBmp("/home/zy/ws/map.bmp");
      if (map_publisher_) {
        map_publisher_->publish(grid.ToRosMsg());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "处理激光扫描数据时发生异常: %s",
                   e.what());
    }
  }

  void odom_event_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // RCLCPP_INFO(
    // rclcpp::get_logger("gazebo_subscriber"),
    // "里程计数据 - 位置: (%.2f, %.2f, %.2f), 姿态: (%.2f, %.2f, %.2f,
    // %.2f)",
    // msg->pose.pose.position.x, msg->pose.pose.position.y,
    // msg->pose.pose.position.z, msg->pose.pose.orientation.x,
    // msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
    // msg->pose.pose.orientation.w);
    odo_.FromRosMsg(msg);
  }

  Pose2D GetOdom() {
    return odom_pose_;
  };
  void PublishResult();

  Gmapping gmapping_;
  rclcpp::TimerBase::SharedPtr main_loop_timer_;

  double freq = 10;
  Pose2D odom_pose_;
  Odom odo_;
  LaserScan laser_scan_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
