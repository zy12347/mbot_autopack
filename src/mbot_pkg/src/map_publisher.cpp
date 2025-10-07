#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class MapPublisher : public rclcpp::Node {
 public:
  MapPublisher() : Node("map_publisher") {
    // 创建地图发布者
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // 设置定时器，定期发布地图
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapPublisher::publish_map, this));

    RCLCPP_INFO(this->get_logger(), "Map publisher node initialized");
  }

 private:
  void publish_map() {
    nav_msgs::msg::OccupancyGrid map_msg;

    // 设置地图元数据
    map_msg.header.stamp = this->get_clock()->now();
    map_msg.header.frame_id = "map";
    map_msg.info.width = 50;                // 地图宽度（单元格数）
    map_msg.info.height = 50;               // 地图高度（单元格数）
    map_msg.info.resolution = 0.5;          // 每个单元格的尺寸（米）
    map_msg.info.origin.position.x = -12.5; // 地图原点
    map_msg.info.origin.position.y = -12.5;
    map_msg.info.origin.orientation.w = 1.0;

    // 初始化地图数据（-1表示未知，0表示空闲，100表示障碍物）
    map_msg.data.resize(map_msg.info.width * map_msg.info.height, 0);

    // 添加一些障碍物
    add_obstacle(map_msg, 10, 10, 5, 5); // x, y, width, height
    add_obstacle(map_msg, 30, 20, 8, 4);
    add_obstacle(map_msg, 20, 35, 6, 6);

    map_pub_->publish(map_msg);
  }

  // 在地图上添加矩形障碍物
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

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  rclcpp::shutdown();
  return 0;
}
