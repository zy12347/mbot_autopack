#include <random>

#include "particle.h"

//局部区域提取,直接遍历不适用bresnham算法
std::vector<std::pair<float, float>> Particle::ScanMap(float max_range) {
  return map.ScanMap(pose, max_range);
}

void Particle::PertubPose() {
  // std::vector<Pose2D> candidate_poses;
  // for (int k = 0; k < K; ++k) {
  //   // 生成满足限制的随机位姿
  //   Pose2D x_j;
  //   // 位置：在 [x_hat.x - delta_p, x_hat.x + delta_p] 内随机
  //   x_j.x = pose.x + uniformRandom(-delta_p, delta_p);
  //   x_j.y = pose.y + uniformRandom(-delta_p, delta_p);
  //   // 角度：在 [x_hat.theta - delta_theta, x_hat.theta + delta_theta] 内随机
  //   x_j.theta = pose.theta + uniformRandom(-delta_theta, delta_theta);
  //   // 角度归一化（确保在 [-π, π] 内）
  //   x_j.theta = normalizeAngle(x_j.theta);

  //   candidate_poses.push_back(x_j);
  // }
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(-delta_p, delta_p);
  pose.x = pose.x + dist(gen);
  pose.y = pose.y + dist(gen);
  float phi = pose.GetPhi() + dist(gen);
  pose.SetPhi(Pose2D::Wrap2PI(phi));
}