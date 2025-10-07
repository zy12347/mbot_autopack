#include "hybrid_astar.h"
#include <cmath>
#include <iostream>

// 规划主函数
std::vector<State> HybridAStar::plan(double start_x, double start_y,
                                     double start_theta, double goal_x,
                                     double goal_y, double goal_theta) {
  // 初始化起点和目标状态
  State start(
      start_x, start_y, normalize_angle(start_theta),
      0.0, // g=0
      heuristic(start_x, start_y, start_theta, goal_x, goal_y, goal_theta));
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  goal_theta_ = normalize_angle(goal_theta);
  // 优先队列（开放列表）：小顶堆（按f=g+h排序）
  std::priority_queue<State, std::vector<State>, std::greater<State>> open_list;
  open_list.push(start);

  // 封闭列表：存储已访问的状态（哈希表）
  std::unordered_map<std::string, bool> closed_list;

  // 目标状态指针
  std::shared_ptr<State> goal_state = nullptr;

  // 最大迭代次数（防止无限循环）
  int max_iter = 10000;
  int iter = 0;

  while (!open_list.empty()) {
    // 取出代价最小的状态
    State current = open_list.top();
    open_list.pop();
    std::cout << "x y: " << current.x << " " << current.y << std::endl;
    iter++;

    // 判断是否到达目标
    double dist_to_goal = std::hypot(current.x - goal_x, current.y - goal_y);
    double theta_diff = std::fabs(normalize_angle(current.theta - goal_theta));
    if (dist_to_goal < goal_dist_thresh_ && theta_diff < goal_theta_thresh_) {
      goal_state = std::make_shared<State>(current);
      break;
    }

    // 若已在封闭列表，跳过
    std::string current_key = current.hash_key();
    if (closed_list.find(current_key) != closed_list.end()) {
      continue;
    }
    closed_list[current_key] = true;

    // 生成后继状态
    std::vector<State> successors = generate_successors(current);
    for (const auto& succ : successors) {
      // 若后继状态未访问，加入开放列表
      std::string succ_key = succ.hash_key();
      if (closed_list.find(succ_key) == closed_list.end()) {
        open_list.push(succ);
      }
    }
  }

  // 回溯路径
  std::vector<State> path;
  if (goal_state != nullptr) {
    std::shared_ptr<State> curr = goal_state;
    while (curr != nullptr) {
      path.push_back(*curr);
      curr = curr->parent;
    }
    // 反转路径（从起点到目标）
    std::reverse(path.begin(), path.end());
    std::cout << "规划成功！路径长度：" << path.size() << " 段" << std::endl;
  } else {
    std::cout << "规划失败，未找到路径" << std::endl;
  }

  return path;
}

// 启发函数
double HybridAStar::heuristic(double x, double y, double theta, double goal_x,
                              double goal_y, double goal_theta) {
  // 位置代价（欧氏距离）
  double dist = std::hypot(x - goal_x, y - goal_y);
  // 朝向代价（角度差）
  double theta_diff = std::fabs(normalize_angle(theta - goal_theta));
  return dist + theta_diff; // 权重可调整
}

// 碰撞检测（简化为点碰撞，实际应检测机器人轮廓）
bool HybridAStar::is_collision(double x, double y) {
  int grid_x = x2idx(x);
  int grid_y = y2idy(y);
  if (grid_x < 0 || grid_x > width_ || grid_y < 0 || grid_y > height_) {
    return true; // 超出地图范围视为碰撞
  }
  // 栅格索引（行优先）
  int index = grid_x + grid_y * width_;
  // 地图值为100时是障碍物
  return (map_.data[index] == 100);
}

// 生成后继状态（基于差速模型）
std::vector<State> HybridAStar::generate_successors(const State& current) {
  std::vector<State> successors;
  // 可能的转向角度（直走、左转、右转）
  std::vector<double> delta_thetas = {
      0,               // 直走
      theta_step_,     // 左转
      -theta_step_,    // 右转
      theta_step_ / 2, // 左转（小步）
      -theta_step_ / 2 // 右转（小步）
  };

  for (double d_theta : delta_thetas) {
    // 计算新朝向
    double new_theta = normalize_angle(current.theta + d_theta);
    // 计算新位置（圆弧轨迹中点）
    double new_x = current.x + step_len_ * cos(current.theta + d_theta / 2);
    double new_y = current.y + step_len_ * sin(current.theta + d_theta / 2);

    // 碰撞检测
    if (is_collision(new_x, new_y)) {
      continue;
    }

    // 新代价（累积路径长度）
    double new_g = current.g + step_len_;
    // 启发代价（假设目标不变，需传入实际目标坐标）
    // 注：实际使用时需保存目标坐标为成员变量，此处简化
    double new_h = heuristic(new_x, new_y, new_theta, goal_x_, goal_y_,
                             goal_theta_); // 实际应调用heuristic()

    // 添加后继状态
    successors.emplace_back(new_x, new_y, new_theta, new_g, new_h,
                            std::make_shared<State>(current));
  }

  return successors;
}

// 角度归一化到[-π, π]
double HybridAStar::normalize_angle(double theta) {
  while (theta > M_PI)
    theta -= 2 * M_PI;
  while (theta < -M_PI)
    theta += 2 * M_PI;
  return theta;
}
