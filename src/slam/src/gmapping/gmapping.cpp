#include "gmapping.h"

#include <cmath>
#include <iostream>
#include <random>

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
  std::cout << "ProcessScan: scan.ranges.size()=" << scan.ranges.size()
            << ", range_min=" << scan.range_min
            << ", range_max=" << scan.range_max << std::endl;

  UpdateSensorData(scan, odom);
  Predict();
  OptimizePose();
  std::cout << "optimize: completed" << std::endl;
  computeAndNormalizeWeights();
  std::cout << "computeAndNormalizeWeights: completed" << std::endl;
  UpdateMap();

  std::cout << "UpdateMap: completed" << std::endl;
  return;
}

void Gmapping::Resample() {
  // TODO: 实现重采样算法
  std::vector<Particle> new_particles;
  std::vector<double> cdf(particles_.size(), 0.0);
  cdf[0] = particles_[0].GetWeight();
  for (size_t i = 1; i < particles_.size(); ++i) {
    cdf[i] = cdf[i - 1] + particles_[i].GetWeight();
  }
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(0.0, 1.0 / particles_.size());
  std::uniform_real_distribution<float> dist_noise(0.0, 0.01);
  float r = dist(gen);
  size_t i = 0;
  for (size_t m = 0; m < particles_.size(); ++m) {
    float U = r + m * (1.0 / particles_.size());
    while (U > cdf[i] && i < particles_.size() - 1) {
      i++;
    }
    Particle new_p = particles_[i];  // 这里会调用拷贝构造函数
    new_p.SetWeight(1.0 / particles_.size());
    Pose2D new_pose = new_p.GetPose();
    // 可选：添加微小噪声避免粒子完全相同（增加多样性）
    new_pose.x += dist_noise(gen);  // 0.01m标准差的噪声
    new_pose.y += dist_noise(gen);
    new_pose.SetPhi(new_pose.GetPhi() + dist_noise(gen));  // 弧度噪声
    new_p.SetPose(new_pose);
    new_particles.push_back(new_p);
  }
  particles_ = new_particles;
}

void Gmapping::UpdateMap() {
  Pose2D best_pose = GetBestEstimate();
  std::cout << "UpdateMap: best_pose=(" << best_pose.x << "," << best_pose.y
            << "," << best_pose.GetPhi() << ")" << std::endl;

  // 更新全局地图
  grid_map_.UpdateCell(best_pose, laser_scan_);

  // 更新所有粒子的地图
  for (auto& particle : particles_) {
    particle.GetGridMap().UpdateCell(best_pose, laser_scan_);
  }

  std::cout << "UpdateMap: updated " << particles_.size() << " particles"
            << std::endl;
}

void Gmapping::computeAndNormalizeWeights() {
  double total_weight = 0.0;
  std::cout << "computeAndNormalizeWeights: starting" << std::endl;
  // 遍历每个粒子计算权重
  for (auto& particle : particles_) {
    double weight = ComputeParticleWeight(particle);
    particle.SetWeight(weight);
    total_weight += weight;
  }
  std::cout << "computeWeights: completed" << std::endl;
  // 归一化权重
  if (total_weight > 0) {
    for (auto& particle : particles_) {
      double normalized = particle.GetWeight() / total_weight;
      particle.SetWeight(normalized);
    }
  }
  std::cout << "NormalizeWeights: completed" << std::endl;
  // 计算N_nerf
  float nerf = 0.0;
  for (auto& particle : particles_) {
    nerf += particle.GetWeight() * particle.GetWeight();
  }
  nerf = 1 / nerf;
  if (nerf < nerf_threshold_) {
    Resample();
  }
  std::cout << "Nerf: completed" << std::endl;
}

float Gmapping::ComputeParticleWeight(Particle& particle) {
  std::cout << "ComputeParticleWeight: starting" << std::endl;
  const GridMap& grid_map = particle.GetGridMap();  // 使用引用避免拷贝
  Pose2D p_pose = particle.GetPose();
  std::cout << "ComputeParticleWeight: grid_map.Raycast: starting" << std::endl;
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
  std::cout << "ComputeParticleWeight grid_map.Raycast: completed" << std::endl;
  return std::exp(log_weight);
}

void Gmapping::OptimizePose() {
  std::cout << "OptimizePose: starting pose optimization" << std::endl;

  std::vector<std::pair<float, float>> points =
      Polar2Cartesian(laser_scan_.ranges);

  // 检查当前激光扫描点云是否为空
  if (points.empty()) {
    std::cout << "Warning: Current laser scan points are empty, skipping ICP"
              << std::endl;
    return;
  }

  std::cout << "OptimizePose: current scan has " << points.size() << " points"
            << std::endl;

  for (auto& p : particles_) {
    std::vector<std::pair<float, float>> points_scan =
        p.ScanMap(laser_scan_.range_max);

    // 检查粒子地图点云是否为空
    if (points_scan.empty()) {
      std::cout
          << "Warning: Particle map points are empty, skipping this particle"
          << std::endl;
      continue;
    }

    Pose2D optimized_pose = scan_matcher_.ICP(points, points_scan, p.GetPose());
    p.SetPose(optimized_pose);
    p.PertubPose();
  }
}

Pose2D Gmapping::GetBestEstimate() {
  // 返回权重最大的粒子位姿
  if (particles_.empty()) {
    // 如果没有粒子，返回默认位姿
    return Pose2D(0.0, 0.0, 0.0);
  }

  double max_weight = 0.0;
  Pose2D best_pose = particles_[0].GetPose();  // 初始化为第一个粒子

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
  for (size_t i = 0; i < ranges.size(); i++) {
    // 过滤无效的距离值
    if (ranges[i] < laser_scan_.range_min ||
        ranges[i] > laser_scan_.range_max) {
      continue;
    }

    // 使用正确的角度计算
    float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
    float x = cos(angle) * ranges[i];
    float y = sin(angle) * ranges[i];
    std::pair<float, float> point(x, y);
    points.emplace_back(point);
  }
  return points;
}