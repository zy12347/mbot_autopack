#include "gmapping.h"

#include <cmath>
#include <iostream>

void Gmapping::Initialize(Pose2D& init_pose) {
  particles_.reserve(gmap_param_.particle_count);
  for (int i = 0; i < gmap_param_.particle_count; i++) {
    Particle p(init_pose);
    p.SetWeight(1.0 / double(gmap_param_.particle_count));
    particles_.emplace_back(p);
  }
}

void Gmapping::Predict() {
  double delta_t = odo_.stamp - last_stamp_time_;
  last_stamp_time_ = odo_.stamp;
  double delta_d = odo_.linear_x * delta_t;
  double delta_theta = odo_.angular_z * delta_t;
  for (auto& p : particles_) {
    Pose2D last_pose = p.GetPose();
    Pose2D cur_pose;
    cur_pose.x =
        last_pose.x + delta_d * cos(last_pose.GetPhi() + delta_theta / 2);
    cur_pose.y =
        last_pose.y + delta_d * sin(last_pose.GetPhi() + delta_theta / 2);
    cur_pose.SetPhi(last_pose.GetPhi() + delta_theta);
    p.SetPose(cur_pose);
  }
}

void Gmapping::ProcessScan(LaserScan& scan, Odom& odom) {
  UpdateSensorData(scan, odom);
  Predict();
  OptimizePose();
  computeAndNormalizeWeights();
  UpdateMap();

  std::cout << scan.range_min << std::endl;

  return;
}

void Gmapping::Resample() {
  // TODO: 实现重采样算法
}

void Gmapping::UpdateMap() {
  Pose2D best_pose = GetBestEstimate();
  grid_map_.UpdateCell(best_pose, laser_scan_);
  // TODO: 使用最佳位姿更新地图
}

void Gmapping::computeAndNormalizeWeights() {
  double total_weight = 0.0;

  // 遍历每个粒子计算权重
  for (auto& particle : particles_) {
    double weight = ComputeParticleWeight(particle);
    particle.SetWeight(weight);
    total_weight += weight;
  }

  // 归一化权重
  if (total_weight > 0) {
    for (auto& particle : particles_) {
      double normalized = particle.GetWeight() / total_weight;
      particle.SetWeight(normalized);
    }
  }
}

float Gmapping::ComputeParticleWeight(Particle& particle) {
  auto grid_map = particle.GetGridMap();
  Pose2D p_pose = particle.GetPose();
  float sigma = 0.05;
  float log_weight = 0;
  for (size_t i = 0; i < laser_scan_.ranges.size(); i++) {
    float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
    float global_angle = p_pose.GetPhi() + angle;

    float distance = grid_map.Raycast(p_pose.x, p_pose.y, global_angle,
                                      laser_scan_.range_max);
    float diff = distance - laser_scan_.ranges[i];
    log_weight += -0.5 * (diff * diff) / (sigma * sigma);
  }
  return std::exp(log_weight);
}

void Gmapping::OptimizePose() {
  std::vector<std::pair<float, float>> points =
      Polar2Cartesian(laser_scan_.ranges);
  for (auto& p : particles_) {
    std::vector<std::pair<float, float>> points_scan =
        p.ScanMap(laser_scan_.range_max);
    Pose2D optimized_pose = scan_matcher_.ICP(points, points_scan, p.GetPose());
    p.SetPose(optimized_pose);
    p.PertubPose();
  }
}

Pose2D Gmapping::GetBestEstimate() {
  // 返回权重最大的粒子位姿
  double max_weight = 0.0;
  Pose2D best_pose;

  for (const auto& p : particles_) {
    if (p.GetWeight() > max_weight) {
      max_weight = p.GetWeight();
      best_pose = p.GetPose();
    }
  }

  return best_pose;
}

std::vector<std::pair<float, float>> Gmapping::Polar2Cartesian(
    std::vector<float>& ranges) {
  std::vector<std::pair<float, float>> points;
  for (int i = 0; i < ranges.size(); i++) {
    float x = cos(Pose2D::DEG2RAD(i)) * ranges[i];
    float y = sin(Pose2D::DEG2RAD(i)) * ranges[i];
    std::pair<float, float> point(x, y);
    points.emplace_back(point);
  }
  return points;
}