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
    map_sub_ptr_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          this->map_callback(msg);
        });

    path_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
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
    // plan_path();
    generateRectangle();
  }
  // 将物理坐标转换为栅格索引

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_msg_ = *msg;
  }
  void plan_path() {
    // 1. 调用路径规划算法（如A*、RRT）生成路径点列表
    HybridAStar hybrid_star(map_msg_);
    std::vector<State> planned_path = hybrid_star.plan(
        start_pose_.pose.position.x, start_pose_.pose.position.y,
        start_pose_angle_, goal_pose_.pose.position.x,
        goal_pose_.pose.position.y, goal_pose_angle_);
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
  }

  void generateRectangle() {
    nav_msgs::msg::Path path_msg;

    // 设置路径头部信息
    path_msg.header.frame_id = "world";               // 路径所在坐标系
    path_msg.header.stamp = this->get_clock()->now(); // 当前时间戳

    // 生成1m×1m矩形路径的四个顶点
    // 从原点(0,0)开始，顺时针生成矩形
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;

    // 点1: 起点 (0, 0)
    waypoints.push_back(create_waypoint(0.0, 0.0, 0.0));

    // 点2: 右移1米 (1, 0)
    waypoints.push_back(create_waypoint(1.0, 0.0, 0));

    // 点3: 上移1米 (1, 1)
    waypoints.push_back(create_waypoint(1.0, 1.0, 0));

    // 点4: 左移1米 (0, 1)
    waypoints.push_back(create_waypoint(0.0, 1.0, 0));

    // 点5: 下移1米回到起点 (0, 0)
    waypoints.push_back(create_waypoint(0.0, 0.0, 0));

    // 为了使路径更平滑，可以在顶点之间添加中间点
    add_intermediate_points(waypoints, 5); // 每个边添加5个中间点

    // 将路点添加到路径消息中
    path_msg.poses = waypoints;

    // 发布路径
    path_publisher_->publish(path_msg);
    for (auto p : path_msg.poses) {
      std::cout << p.pose.position.x << " " << p.pose.position.y << std::endl;
    }
    RCLCPP_DEBUG(this->get_logger(), "已发布矩形路径，包含%d个路点",
                 path_msg.poses.size());
  }

  geometry_msgs::msg::PoseStamped create_waypoint(double x, double y,
                                                  double z) {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = "world";
    waypoint.header.stamp = this->get_clock()->now();

    // 设置位置
    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    waypoint.pose.position.z = z; // 2D路径可以设为0

    // 设置姿态（默认朝向）
    waypoint.pose.orientation.w = 1.0; // 单位四元数，表示无旋转

    return waypoint;
  }

  void add_intermediate_points(
      std::vector<geometry_msgs::msg::PoseStamped>& waypoints, int num_points) {
    if (waypoints.size() < 2 || num_points <= 0)
      return;

    std::vector<geometry_msgs::msg::PoseStamped> new_waypoints;

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
      // 添加当前点
      new_waypoints.push_back(waypoints[i]);

      // 计算当前点和下一个点之间的步长
      double dx =
          (waypoints[i + 1].pose.position.x - waypoints[i].pose.position.x) /
          (num_points + 1);
      double dy =
          (waypoints[i + 1].pose.position.y - waypoints[i].pose.position.y) /
          (num_points + 1);
      double dz =
          (waypoints[i + 1].pose.position.z - waypoints[i].pose.position.z) /
          (num_points + 1);

      // 添加中间点
      for (int j = 1; j <= num_points; ++j) {
        geometry_msgs::msg::PoseStamped intermediate = waypoints[i];
        intermediate.pose.position.x += dx * j;
        intermediate.pose.position.y += dy * j;
        intermediate.pose.position.z += dz * j;
        new_waypoints.push_back(intermediate);
      }
    }

    // 添加最后一个点
    new_waypoints.push_back(waypoints.back());

    // 更新路点列表
    waypoints = new_waypoints;
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      init_pose_sub_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_pose_sub_ptr_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_ptr_;

  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  nav_msgs::msg::OccupancyGrid map_msg_;
  float start_pose_angle_;
  float goal_pose_angle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
