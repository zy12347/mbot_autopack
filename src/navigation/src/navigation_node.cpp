#include <cstdio>
#include "Astar.h"
#include "RandomTree.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // 新增头文件
#include "hybrid_astar.h"
#include "mbot_interface/mbot_common.h"
#include "nav_msgs/msg/path.hpp" // Path 消息头文件

class NavigationManager : public rclcpp ::Node {
 public:
  NavigationManager() : Node("navigation") {
    init_pose_sub_ptr_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
          this->init_pose_event_callback(msg);
        });
    goal_pose_sub_ptr_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            [this](const geometry_msgs::msg::PoseStamped& msg) {
              this->goal_pose_event_callback(msg);
            });

    click_subscriber_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 1,
            [this](const geometry_msgs::msg::PointStamped& msg) {
              this->click_callback(msg);
            });

    path_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    path_publisher1_ =
        this->create_publisher<nav_msgs::msg::Path>("/planned_path1", 10);

    map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     [this]() { this->publish_map(); });
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
  void init_pose_event_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    start_pose_.header = msg.header;
    start_pose_.pose = msg.pose.pose;
    start_pose_angle_ = 2 * std::atan2(start_pose_.pose.orientation.z,
                                       start_pose_.pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "收到起始点: x=%.2f, y=%.2f, theta:%.2f",
                start_pose_.pose.position.x, start_pose_.pose.position.y,
                start_pose_angle_);
  }

  void goal_pose_event_callback(const geometry_msgs::msg::PoseStamped& msg) {
    goal_pose_ = msg;
    goal_pose_angle_ = 2 * std::atan2(goal_pose_.pose.orientation.z,
                                      goal_pose_.pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "收到目标点: x=%.2f, y=%.2f, theta:%.2f",
                goal_pose_.pose.position.x, goal_pose_.pose.position.y,
                goal_pose_angle_);
    plan_path();
  }

  // 点击回调：处理鼠标点击事件
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

  void plan_path() {
    // 1. 调用路径规划算法（如A*、RRT）生成路径点列表
    // 假设路径点存储在 std::vector<geometry_msgs::msg::PoseStamped> path_poses
    // 中
    // std::vector<geometry_msgs::msg::PoseStamped> path_poses =
    //     generate_path_points();
    // uint8_t* map_data_ =
    //     new uint8_t[map_msg_.info.width * map_msg_.info.height]();
    // for (int i = 0; i < map_msg_.info.width * map_msg_.info.height; i++) {
    //   map_data_[i] = map_msg_.data[i] == 0 ? 100 : 0;
    // }
    // Astar astar(map_msg_.info.width, map_msg_.info.height, map_data_);
    HybridAStar hybrid_star(map_msg_);
    std::vector<State> planned_path = hybrid_star.plan(
        start_pose_.pose.position.x, start_pose_.pose.position.y,
        start_pose_angle_, goal_pose_.pose.position.x,
        goal_pose_.pose.position.y, goal_pose_angle_);
    // RandomTree rrt(map_msg_.info.width, map_msg_.info.height, map_data_);
    // std::vector<std::pair<int, int>> planned_path = astar.PlanPath(
    //     x2idx(start_pose_.pose.position.x),
    //     y2idy(start_pose_.pose.position.y),
    //     x2idx(goal_pose_.pose.position.x),
    //     y2idy(goal_pose_.pose.position.y));
    std::vector<geometry_msgs::msg::PoseStamped> path_poses;
    for (auto p : planned_path) {
      geometry_msgs::msg::PoseStamped pose;

      // 设置坐标系和时间戳（与地图一致）
      pose.header.frame_id = "map";                 // 必须与地图frame_id一致
      pose.header.stamp = this->get_clock()->now(); // 当前时间戳
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.position.z = 0.0;

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(p.theta / 2);
      pose.pose.orientation.w = std::cos(p.theta / 2);
      path_poses.push_back(pose);
    }
    // 2. 封装为 nav_msgs::msg::Path
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map"; // 必须与RViz2的Fixed Frame一致
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.poses = path_poses; // 将路径点赋值给 Path 的 poses 字段

    // 3. 发布路径消息
    path_publisher_->publish(path_msg);

    uint8_t* map_data_ =
        new uint8_t[map_msg_.info.width * map_msg_.info.height]();
    for (int i = 0; i < map_msg_.info.width * map_msg_.info.height; i++) {
      map_data_[i] = map_msg_.data[i] == 0 ? 100 : 0;
    }
    Astar astar(map_msg_.info.width, map_msg_.info.height, map_data_);
    std::vector<std::pair<int, int>> planned_path1 = astar.PlanPath(
        x2idx(start_pose_.pose.position.x), y2idy(start_pose_.pose.position.y),
        x2idx(goal_pose_.pose.position.x), y2idy(goal_pose_.pose.position.y));
    // RandomTree rrt(map_msg_.info.width, map_msg_.info.height, map_data_);
    // std::vector<std::pair<int, int>> planned_path = astar.PlanPath(
    //     x2idx(start_pose_.pose.position.x),
    //     y2idy(start_pose_.pose.position.y),
    //     x2idx(goal_pose_.pose.position.x),
    //     y2idy(goal_pose_.pose.position.y));
    std::vector<geometry_msgs::msg::PoseStamped> path_poses1;
    for (auto p : planned_path1) {
      geometry_msgs::msg::PoseStamped pose;

      // 设置坐标系和时间戳（与地图一致）
      pose.header.frame_id = "map";                 // 必须与地图frame_id一致
      pose.header.stamp = this->get_clock()->now(); // 当前时间戳
      pose.pose.position.x = idx2x(p.first);
      pose.pose.position.y = idy2y(p.second);
      pose.pose.position.z = 0.0;

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 0;
      path_poses1.push_back(pose);
    }
    // 2. 封装为 nav_msgs::msg::Path
    nav_msgs::msg::Path path_msg1;
    path_msg1.header.frame_id = "map"; // 必须与RViz2的Fixed Frame一致
    path_msg1.header.stamp = this->get_clock()->now();
    path_msg1.poses = path_poses1; // 将路径点赋值给 Path 的 poses 字段
    path_publisher1_->publish(path_msg1);
    RCLCPP_INFO(this->get_logger(), "已发布路径，包含 %d 个点",
                (int)path_poses1.size());
  }

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

    map_publisher_->publish(map_msg_);
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
  // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
  // map_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher1_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      click_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      init_pose_sub_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_pose_sub_ptr_;

  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  nav_msgs::msg::OccupancyGrid map_msg_;
  std::vector<std::pair<int, int>> click_obs_;
  float start_pose_angle_;
  float goal_pose_angle_;
  // GridMap map_;
  // std::vector<signed char> map_data_;
  // int width_;
  // int height_;
  // float resolution_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  // printf("hello world navigation package\n");
  // cv::Mat img = cv::imread("img/grid_map.bmp", 0);
  // if (img.empty()) {
  //   printf("cannot read image file: grid_map.bmp\n");
  //   return -1;
  // }
  return 0;
}
