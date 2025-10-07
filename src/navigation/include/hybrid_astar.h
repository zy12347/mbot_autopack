#pragma once
#include <cmath>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"

// 状态结构体：x,y(米), theta(弧度), g(代价), h(启发)
struct State {
  double x, y, theta;
  double g, h;
  std::shared_ptr<State> parent; // 父节点（用于回溯路径）

  // 构造函数
  State(double x_, double y_, double theta_, double g_, double h_,
        std::shared_ptr<State> parent_ = nullptr)
      : x(x_), y(y_), theta(theta_), g(g_), h(h_), parent(parent_) {}

  // 用于优先队列排序（代价低的先出队）
  bool operator>(const State& other) const {
    return (g + h) > (other.g + other.h); // 小顶堆
  }

  // 用于哈希表（封闭列表）的键值生成
  std::string hash_key() const {
    // 离散化坐标和角度（精度：x,y保留2位小数，theta保留1位小数）
    int ix = static_cast<int>(x / 0.25);
    int iy = static_cast<int>(y / 0.25);

    // 角度精度：与转向步长匹配（例如theta_step_=30°，则30°为一档）
    double theta_step =
        M_PI / 36; // 30度（与generate_successors中的delta_thetas匹配）
    int itheta = static_cast<int>(theta / theta_step + 0.5); // 四舍五入

    return std::to_string(ix) + "," + std::to_string(iy) + "," +
           std::to_string(itheta);
    // return std::to_string(int(x * 100)) + "," + std::to_string(int(y * 100));
  }
};

class HybridAStar {
 public:
  HybridAStar(const nav_msgs::msg::OccupancyGrid& map) : map_(map) {
    // 初始化地图参数
    resolution_ = map.info.resolution;
    origin_x_ = map.info.origin.position.x;
    origin_y_ = map.info.origin.position.y;
    width_ = map.info.width;
    height_ = map.info.height;
  }

  // 规划路径：start(x,y,theta) -> goal(x,y,theta)
  std::vector<State> plan(double start_x, double start_y, double start_theta,
                          double goal_x, double goal_y, double goal_theta);

 private:
  // 地图数据
  nav_msgs::msg::OccupancyGrid map_;
  double resolution_;          // 栅格分辨率（米/栅格）
  double origin_x_, origin_y_; // 地图原点（米）
  int width_, height_;         // 地图尺寸（栅格数）

  // 机器人参数（可根据实际调整）
  const double step_len_ = 0.5;         // 步长（米）
  const double theta_step_ = M_PI / 36; // 转向角度增量（5°）
  const double min_turn_radius_ = 0.2;  // 最小转弯半径（米）

  // 目标阈值（位置误差<0.1m，角度误差<5°）
  const double goal_dist_thresh_ = 0.25;
  const double goal_theta_thresh_ = M_PI / 18;

  float goal_x_;
  float goal_y_;
  float goal_theta_;

  // 启发函数：计算当前状态到目标的估计代价
  double heuristic(double x, double y, double theta, double goal_x,
                   double goal_y, double goal_theta);

  // 物理坐标转栅格索引
  float idx2x(int idx) {
    return float(idx) * map_.info.resolution + map_.info.origin.position.x;
  };
  float idy2y(int idy) {
    return float(idy) * map_.info.resolution + map_.info.origin.position.y;
  };
  int x2idx(float x) {
    return static_cast<int>((x - map_.info.origin.position.x) /
                            map_.info.resolution);
  };
  int y2idy(float y) {
    return static_cast<int>((y - map_.info.origin.position.y) /
                            map_.info.resolution);
    // int(idy * map_msg_.info.resolution + map_msg_.info.origin.position.y);
  };

  // 碰撞检测：判断机器人在(x,y,theta)处是否碰撞
  bool is_collision(double x, double y);

  // 生成后继状态（基于运动学模型）
  std::vector<State> generate_successors(const State& current);

  // 角度归一化到[-π, π]
  double normalize_angle(double theta);
};