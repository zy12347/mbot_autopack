#pragma once
#include "mbot_interface/mbot_common.h"

class Particle {
 public:
  Particle(Pose2D& pose, double map_resolution = 0.05, int map_width = 100,
           int map_height = 100)
      : map(map_width, map_height, map_resolution),
        weight(1.0),
        pose(pose),
        K(10),
        delta_p(0.1),
        delta_theta(1){};

  // 拷贝构造函数
  Particle(const Particle& other)
      : map(other.map),
        weight(other.weight),
        pose(other.pose),
        K(other.K),
        delta_p(other.delta_p),
        delta_theta(other.delta_theta) {}

  // 赋值运算符
  Particle& operator=(const Particle& other) {
    if (this != &other) {
      pose = other.pose;
      map = other.map;
      weight = other.weight;
      K = other.K;
      delta_p = other.delta_p;
      delta_theta = other.delta_theta;
    }
    return *this;
  }
  void SetWeight(double w) {
    weight = w;
  };
  double GetWeight() const {
    return weight;
  };
  void SetPose(Pose2D& pose) {
    this->pose = pose;
  };
  Pose2D GetPose() const {
    return pose;
  };
  std::vector<std::pair<float, float>> ScanMap(float max_range);
  const GridMap& GetGridMap() const {
    return map;
  };
  GridMap& GetGridMap() {
    return map;
  };
  void PerturbPose();

 private:
  GridMap map;
  double weight = 1.0;
  Pose2D pose;
  int K = 10;
  float delta_p = 0.1;
  float delta_theta = 1;
};