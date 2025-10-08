#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gmapping/gmapping.h"
class SlamManager : public rclcpp::Node {
 public:
  SlamManager() : Node("slam") {
    // publisher_ = this->create_publisher<mbot_interface::msg::Person>("", 10);

    // laser_subscriber_ =
    // this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "mbot/laser_plugin/out", 10,
    //     [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    //       this->laser_event_callback(msg);
    //     });

    // timer_ =
    // this->create_wall_timer(std::chrono::seconds(1),[this](){this->timer_callback();});

    // odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "mbot/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    //     {
    //       this->odom_event_callback(msg);
    //     });

    // auto period = std::chrono::duration<double>(1.0 / freq);
    // main_loop_timer_ = this->create_wall_timer(
    //     period,
    //     std::bind(&SlamManager::main_loop, this)
    // );

    // 初始化发布者
    map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    map_publisher_auto_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     [this]() { this->publish_map(); });

    click_subscriber_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 1,
            [this](const geometry_msgs::msg::PointStamped& msg) {
              this->click_callback(msg);
            });

    pose_publisher_ =
        this->create_publisher<mbot_interface::msg::Pose2D>("robot_pose", 10);

    RCLCPP_INFO(this->get_logger(), "SLAM节点初始化完成,主循环频率: %.1f Hz",
                freq);

    // 初始化Gmapping，设置初始位姿
    // RCLCPP_INFO(this->get_logger(), "Gmapping初始化");
    // Pose2D init_pose(0.0, 0.0, 0.0);
    // RCLCPP_INFO(this->get_logger(), "Gmapping初始化");
    // gmapping_.Initialize(init_pose);
  }
  float idx2x(int idx) {
    return float(idx) * map_msg_.info.resolution +
           map_msg_.info.origin.position.x;
  };
  float idy2y(int idy) {
    return float(idy) * map_msg_.info.resolution +
           map_msg_.info.origin.position.y;
  };
  int x2idx(float x) {
    return static_cast<int>((x - map_msg_.info.origin.position.x) /
                            map_msg_.info.resolution);
  };
  int y2idy(float y) {
    return static_cast<int>((y - map_msg_.info.origin.position.y) /
                            map_msg_.info.resolution);
    // int(idy * map_msg_.info.resolution + map_msg_.info.origin.position.y);
  };

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  rclcpp::Publisher<mbot_interface::msg::Pose2D>::SharedPtr pose_publisher_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      map_publisher_auto_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      click_subscriber_;

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
      // grid.SaveAsBmp("/home/zy/ws/map.bmp");
      if (map_publisher_) {
        GridMap& grid = gmapping_.GetGridMap();
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

  void publish_map() {
    // 设置地图元数据
    map_msg_.header.stamp = this->get_clock()->now();
    map_msg_.header.frame_id = "map";
    map_msg_.info.width = 500;           // 地图宽度（单元格数）
    map_msg_.info.height = 500;          // 地图高度（单元格数）
    map_msg_.info.resolution = 0.05;     // 每个单元格的尺寸（米）
    map_msg_.info.origin.position.x = 0; // 地图原点
    map_msg_.info.origin.position.y = 0;
    map_msg_.info.origin.orientation.w = 1.0;

    // 初始化地图数据（-1表示未知，0表示空闲，100表示障碍物）
    map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height, 0);

    // 添加一些障碍物
    add_obstacle(map_msg_, 100, 100, 50, 50); // x, y, width, height
    add_obstacle(map_msg_, 300, 200, 80, 40);
    add_obstacle(map_msg_, 200, 350, 60, 60);
    for (auto obs : click_obs_) {
      add_obstacle(map_msg_, obs.first, obs.second, 10, 10);
    }

    map_publisher_auto_->publish(map_msg_);
  }
  void add_obstacle(nav_msgs::msg::OccupancyGrid& map, int x, int y, int width,
                    int height) {
    for (int i = x; i < x + width && i < map.info.width; i++) {
      for (int j = y; j < y + height && j < map.info.height; j++) {
        int index = i + j * map.info.width;
        if (index >= 0 && index < map.data.size()) {
          map.data[index] = 100;
        }
      }
    }
  }
  void click_callback(const geometry_msgs::msg::PointStamped click) {
    // 检查坐标系是否匹配
    if (click.header.frame_id != map_msg_.header.frame_id) {
      RCLCPP_WARN(this->get_logger(), "点击坐标与地图坐标系不匹配: %s vs %s",
                  click.header.frame_id.c_str(),
                  map_msg_.header.frame_id.c_str());
      return;
    }

    // 将物理坐标转换为栅格索引
    int grid_x = x2idx(click.point.x);

    int grid_y = y2idy(click.point.y);

    // 计算栅格在数据数组中的索引
    int index = grid_x + grid_y * map_msg_.info.width;

    // 修改栅格值为100（障碍物）
    if (index >= 0 && index < map_msg_.data.size()) {
      // map_msg_.data[index] = 100;
      click_obs_.emplace_back(grid_x, grid_y);
      RCLCPP_INFO(this->get_logger(), "已在栅格(%d, %d)添加障碍物", grid_x,
                  grid_y);

      // 更新时间戳并发布修改后的地图
      map_msg_.header.stamp = this->get_clock()->now();
      publish_map();
    }
  }

  Gmapping gmapping_;
  geometry_msgs::msg::PoseStamped cur_pose_;
  rclcpp::TimerBase::SharedPtr main_loop_timer_;
  nav_msgs::msg::OccupancyGrid map_msg_;
  std::vector<std::pair<int, int>> click_obs_;
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
