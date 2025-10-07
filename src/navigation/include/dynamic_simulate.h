#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

// using namespace std;

class State {
 public:
  double x;     // x坐标 (m)
  double y;     // y坐标 (m)
  double theta; // 航向角 (rad)
  double v;     // 线速度 (m/s)
  double omega; // 角速度 (rad/s)

  State(double x = 0, double y = 0, double theta = 0, double v = 0,
        double omega = 0)
      : x(x), y(y), theta(theta), v(v), omega(omega) {}
};

class Robot {
 public:
  Robot(double x = 0, double y = 0, double theta = 0, double v = 0,
        double omega = 0, double v_max = 0.5, double omega_max = 0.3,
        int radius = 0.05)
      : state_(x, y, theta, v, omega),
        v_max_(v_max),
        omega_max_(omega_max),
        radius(radius) {
    // state_ = State(x, y, theta, v, omega); // 初始位置在(5,5)，静止
    color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256); // 随机颜色
  }
  void randomWalk(double dt) {
    // 随机调整速度（在约束范围内）
    state_.v += (rand() % 200 - 100) / 1000.0; // [-0.1, 0.1] 速度增量
    state_.v = std::max(0.0, std::min(v_max_, state_.v));

    state_.omega += (rand() % 200 - 100) / 1000.0; // [-0.1, 0.1] 角速度增量
    state_.omega = std::max(-omega_max_, std::min(omega_max_, state_.omega));

    // 更新位置和航向角
    state_.x += state_.v * cos(state_.theta) * dt;
    state_.y += state_.v * sin(state_.theta) * dt;
    state_.theta += state_.omega * dt;

    // 角度归一化到[-π, π]
    state_.theta = atan2(sin(state_.theta), cos(state_.theta));

    // 边界反弹（假设环境范围为0~10m）
    if (state_.x < 0 || state_.x > 10) {
      state_.x = std::max(0.0, std::min(10.0, state_.x));
      state_.theta = M_PI - state_.theta; // 反弹
    }
    if (state_.y < 0 || state_.y > 10) {
      state_.y = std::max(0.0, std::min(10.0, state_.y));
      state_.theta = -state_.theta; // 反弹
    }
  }

  void move(double dt) {
    // 更新位置和航向角
    state_.x += state_.v * cos(state_.theta) * dt;
    state_.y += state_.v * sin(state_.theta) * dt;
    state_.theta += state_.omega * dt;

    // 角度归一化到[-π, π]
    state_.theta = atan2(sin(state_.theta), cos(state_.theta));

    // 边界反弹（假设环境范围为0~10m）
    if (state_.x < 0 || state_.x > 10) {
      state_.x = std::max(0.0, std::min(10.0, state_.x));
      state_.theta = M_PI - state_.theta; // 反弹
    }
    if (state_.y < 0 || state_.y > 10) {
      state_.y = std::max(0.0, std::min(10.0, state_.y));
      state_.theta = -state_.theta; // 反弹
    }
  }

  double getV() const {
    return state_.v;
  }

  double getOmega() const {
    return state_.omega;
  }

  double getX() const {
    return state_.x;
  }

  double getY() const {
    return state_.y;
  }

  double getTheta() const {
    return state_.theta;
  }

  double getRadius() const {
    return radius;
  }

  cv::Scalar getColor() const {
    return color;
  }

  bool isCollision();
  std::vector<std::pair<int, int>> pathPlan();

  void pathFollow();

 private:
  std::vector<std::pair<int, int>> path_;
  double v_max_;     // 最大速度 (m/s)
  double omega_max_; // 最大角速度 (rad/s)
  State state_;
  double radius;    // 障碍物半径 (m)
  cv::Scalar color; // 可视化颜色

  uint8_t* map_data_;
};

template <typename T>
class DynamicEnvironment {
 private:
  std::vector<T> obstacles; // 动态障碍物列表
  T robot;                  // 机器人状态
  T goal;                   // 目标点
  //   LatticeGenerator lattice_gen;      // Lattice生成器
  double dt; // 时间步长 (s)
  double resolution_;
  uint8_t* map_data_ = nullptr;
  uint8_t width_ = 500;
  uint8_t height_ = 500;

 public:
  DynamicEnvironment(double dt = 0.1, uint8_t* map_data = nullptr,
                     uint8_t width = 500, uint8_t height = 500,
                     double resolution = 0.05)
      : dt(dt),
        map_data_(map_data),
        width_(width),
        height_(height),
        resolution_(resolution) {
    // ,lattice_gen(0, world_size, 0, world_size) {
  }

  void setRobotState(const T& state) {
    robot = state;
  }

  void setGoal(const T& g) {
    goal = g;
  }
  // 添加动态障碍物
  void addObstacles(int num) {
    for (int i = 0; i < num; ++i) {
      // 在环境中随机生成障碍物（避开起点和目标）
      double x, y;
      do {
        x = (rand() % 100) / 10.0; // 0~10m
        y = (rand() % 100) / 10.0;
      } while (hypot(x - robot.getX(), y - robot.getY()) < 1.0 ||
               hypot(x - goal.getX(), y - goal.getY()) < 1.0);

      obstacles.emplace_back(x, y);
    }
  }

  // 更新环境状态
  void update() {
    // 更新障碍物
    for (auto& obs : obstacles) {
      obs.randomWalk(dt);
    }
    robot.move(dt);
  }

  // 可视化环境
  void visualize() {
    // 创建图像（缩放100倍显示）
    // int scale = 50;
    // cv::Mat img(world_size * scale, world_size * scale, CV_8UC3,
    //             cv::Scalar(255, 255, 255));
    cv::Mat img(height_, width_, CV_8UC3);
    // memcpy(img.data, map_data_, height_ * width_ * sizeof(uint8_t));
    for (int i = 0; i < width_ * height_; i++) {
      if (map_data_[i] == 0) {
        img.data[i * 3] = 0;
        img.data[i * 3 + 1] = 0;
        img.data[i * 3 + 2] = 0;
      }
    }
    // 绘制目标点
    cv::circle(img,
               cv::Point(goal.getX() / resolution_, goal.getY() / resolution_),
               15, cv::Scalar(0, 0, 255), -1);

    // 绘制机器人
    cv::circle(
        img, cv::Point(robot.getX() / resolution_, robot.getY() / resolution_),
        10, cv::Scalar(255, 0, 0), -1);
    // 绘制机器人朝向
    double arrow_len = 2;
    double arrow_x = robot.getX() + arrow_len * cos(robot.getTheta());
    double arrow_y = robot.getY() + arrow_len * sin(robot.getTheta());
    cv::arrowedLine(
        img, cv::Point(robot.getX() / resolution_, robot.getY() / resolution_),
        cv::Point(arrow_x / resolution_, arrow_y / resolution_),
        cv::Scalar(0, 255, 0), 2);

    // 绘制动态障碍物
    for (const auto& obs : obstacles) {
      cv::circle(img,
                 cv::Point(obs.getX() / resolution_, obs.getY() / resolution_),
                 obs.getRadius() / resolution_, obs.getColor(), -1);
      // 绘制障碍物速度方向
      double obs_arrow_len = obs.getRadius() * 2;
      double obs_arrow_x = obs.getX() + obs_arrow_len * cos(obs.getTheta());
      double obs_arrow_y = obs.getY() + obs_arrow_len * sin(obs.getTheta());
      cv::arrowedLine(
          img, cv::Point(obs.getX() / resolution_, obs.getY() / resolution_),
          cv::Point(obs_arrow_x / resolution_, obs_arrow_y / resolution_),
          cv::Scalar(0, 0, 0), 1);
    }

    // 生成并绘制Lattice状态格子（机器人周围）
    // vector<State> lattice =
    // lattice_gen.generateLocalLattice(robot_state, 1.5); for (const auto& s
    // : lattice) {
    //   cv::circle(img, cv::Point(s.x, s.y), 2, cv::Scalar(100, 100, 100),
    //   1);
    // }

    // // 生成并绘制一条示例Lattice轨迹（机器人到前方某点）
    // if (!lattice.empty()) {
    //   State target_state = lattice[100]; // 选取一个格子点作为目标
    //   vector<State> trajectory =
    //       lattice_gen.generateTrajectory(robot_state, target_state);
    //   for (size_t i = 1; i < trajectory.size(); ++i) {
    //     cv::line(
    //         img,
    //         cv::Point(trajectory[i - 1].x * scale, trajectory[i - 1].y *
    //         scale), cv::Point(trajectory[i].x * scale, trajectory[i].y *
    //         scale), cv::Scalar(0, 255, 255), 2);
    //   }
    //   }

    cv::imshow("Dynamic Environment with Lattice", img);
    cv::waitKey(50); // 控制显示帧率
  }

  // 运行模拟器
  void run(int steps = 1000) {
    for (int i = 0; i < steps; ++i) {
      update();
      visualize();
      // 检查是否到达目标
      if (hypot(robot.getX() - goal.getX(), robot.getY() - goal.getY()) <
          0.05) {
        std::cout << "到达目标点！" << std::endl;
        break;
      }
    }
    cv::waitKey(0); // 最后一帧停留
  }
};