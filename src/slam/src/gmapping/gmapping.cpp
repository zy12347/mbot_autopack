#include "gmapping.h"

#include <cmath>
#include <iostream>
#include <random>

Gmapping::Gmapping() {
  gmap_param_.particle_count = 30;
  gmap_param_.map_resolution = 0.05;
  // gmap_param_.map_size = 10000;
  // gmap_param_.map_origin = Pose2D(0.0, 0.0, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("gmapping"), "Gmapping constructor called");
  grid_map_ = GridMap(100, 100, 0.05);
  RCLCPP_INFO(rclcpp::get_logger("gmapping"), "grid_map_ initialized");
}

Gmapping::Gmapping(const Gmapping& other)
    : grid_map_(other.grid_map_),
      particle_num(other.particle_num),
      nerf_threshold_(other.nerf_threshold_),
      last_stamp_time_(other.last_stamp_time_),
      particles_(other.particles_),
      gmap_param_(other.gmap_param_),
      laser_scan_(other.laser_scan_),
      odo_(other.odo_),
      scan_matcher_(other.scan_matcher_) {
  RCLCPP_INFO(rclcpp::get_logger("gmapping"),
              "Gmapping copy constructor called");
}

Gmapping& Gmapping::operator=(const Gmapping& other) {
  if (this != &other) {
    RCLCPP_INFO(rclcpp::get_logger("gmapping"),
                "Gmapping assignment operator called");
    grid_map_ = other.grid_map_;
    particle_num = other.particle_num;
    nerf_threshold_ = other.nerf_threshold_;
    last_stamp_time_ = other.last_stamp_time_;
    particles_ = other.particles_;
    gmap_param_ = other.gmap_param_;
    laser_scan_ = other.laser_scan_;
    odo_ = other.odo_;
    scan_matcher_ = other.scan_matcher_;
  }
  return *this;
}

Gmapping::~Gmapping() {
  RCLCPP_INFO(rclcpp::get_logger("gmapping"), "Gmapping destructor called");
}

void Gmapping::Initialize(Pose2D& init_pose) {
  RCLCPP_INFO(rclcpp::get_logger("gmapping"), "Initialize called");
  particles_.reserve(gmap_param_.particle_count);
  RCLCPP_INFO(rclcpp::get_logger("gmapping"), "particles reserved count: %d",
              gmap_param_.particle_count);
  for (int i = 0; i < gmap_param_.particle_count; i++) {
    Particle p(init_pose);
    p.SetWeight(1.0 / double(gmap_param_.particle_count));
    particles_.emplace_back(p);
  }
  RCLCPP_INFO(rclcpp::get_logger("gmapping"), "particles initialized");
}

void Gmapping::Predict() {
  // 确保时间差不为负数
  uint64_t delta_t = odo_.stamp - last_stamp_time_; // nano
  float delta_t_s = delta_t / 1e9;
  last_stamp_time_ = odo_.stamp;
  float delta_d = odo_.linear_x * delta_t_s;
  float delta_phi = odo_.angular_z * delta_t_s;
  RCLCPP_INFO(rclcpp::get_logger("gmapping Predict"),
              "delta_t:%f delta_d:%f delta_phi:%f", delta_t_s, delta_d,
              delta_phi);
  for (auto& p : particles_) {
    Pose2D last_pose = p.GetPose();
    Pose2D cur_pose;
    cur_pose.x =
        last_pose.x + delta_d * cos(last_pose.GetPhi() + delta_phi / 2);
    cur_pose.y =
        last_pose.y + delta_d * sin(last_pose.GetPhi() + delta_phi / 2);
    cur_pose.SetPhi(last_pose.GetPhi() + delta_phi);
    p.SetPose(cur_pose);
  }
  // RCLCPP_INFO(rclcpp::get_logger("gmapping Predict"), "test");
}

void Gmapping::ProcessScan(LaserScan& scan, Odom& odom) {
  UpdateSensorData(scan, odom);

  Predict();

  auto end_time1 = std::chrono::high_resolution_clock::now(); // 记录结束时间

  OptimizePose();
  auto end_time2 = std::chrono::high_resolution_clock::now(); // 记录结束时间
  auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(
                       end_time2 - end_time1)
                       .count();
  RCLCPP_INFO(rclcpp::get_logger("gmapping"), "OptimizePose took %ld ms",
              duration2);
  RCLCPP_INFO(rclcpp::get_logger("gmapping Predict"), "ComputeWeight");
  ComputeAndNormalizeWeights();
  auto end_time3 = std::chrono::high_resolution_clock::now(); // 记录结束时间
  auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(
                       end_time3 - end_time2)
                       .count();
  RCLCPP_INFO(rclcpp::get_logger("gmapping"),
              "ComputeAndNormalizeWeights took %ld ms", duration3);
  RCLCPP_INFO(rclcpp::get_logger("gmapping Predict"), "UpdateMap");
  UpdateMap();
  auto end_time4 = std::chrono::high_resolution_clock::now(); // 记录结束时间
  auto duration4 = std::chrono::duration_cast<std::chrono::milliseconds>(
                       end_time4 - end_time3)
                       .count();
  RCLCPP_INFO(rclcpp::get_logger("gmapping"), "UpdateMap took %ld ms",
              duration4);
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
    Particle new_p = particles_[i]; // 这里会调用拷贝构造函数
    new_p.SetWeight(1.0 / particles_.size());
    Pose2D new_pose = new_p.GetPose();
    // 可选：添加微小噪声避免粒子完全相同（增加多样性）
    new_pose.x += dist_noise(gen); // 0.01m标准差的噪声
    new_pose.y += dist_noise(gen);
    new_pose.SetPhi(new_pose.GetPhi() + dist_noise(gen)); // 弧度噪声
    new_p.SetPose(new_pose);
    new_particles.push_back(new_p);
  }
  particles_ = new_particles;
}

void Gmapping::UpdateMap() {
  Pose2D best_pose = GetBestEstimate();
  std::cout << "UpdateMap: best_pose=(" << best_pose.x << "," << best_pose.y
            << "," << best_pose.GetPhi() << ")" << std::endl;
  std::cout << "UpdateMap: laser_scan_=(" << laser_scan_.ranges[0] << ","
            << laser_scan_.ranges[1] << ",...) " << std::endl;
  bool need_extend = CheckReachEdge(best_pose);
  if (need_extend) {
    grid_map_.ExtendMap();
    RCLCPP_INFO(
        rclcpp::get_logger("Gmapping"),
        "Reach Max Map, EXTEND width:%d height:%d origin_x:%f origin_y:%f",
        grid_map_.GetWidth(), grid_map_.GetHeight(), grid_map_.GetOriginX(),
        grid_map_.GetOriginY());
  }
  // 更新全局地图
  grid_map_.UpdateCell(best_pose, laser_scan_);

  for (size_t i = 0; i < particles_.size(); ++i) {
    Particle& p = particles_[i];
    if (need_extend) {
      p.GetGridMap().ExtendMap();
    }
    p.GetGridMap().UpdateCell(p.GetPose(), laser_scan_);
  }
  // auto update_map = [this](size_t start, size_t end, bool need_extend) {
  //   for (size_t i = start; i < end; ++i) {
  //     Particle& p = particles_[i];
  //     if (need_extend) {
  //       p.GetGridMap().ExtendMap();
  //     }
  //     p.GetGridMap().UpdateCell(p.GetPose(), laser_scan_);
  //   }
  // };
  // std::thread thread1(update_map, 0, particles_.size() / 2, need_extend);
  // std::thread thread2(update_map, particles_.size() / 2, particles_.size(),
  //                     need_extend);

  // // 等待线程完成
  // thread1.join();
  // thread2.join();

  // 更新所有粒子的地图
  // for (auto& particle : particles_) {
  //   if (need_extend) {
  //     particle.GetGridMap().ExtendMap();
  //   }
  //   particle.GetGridMap().UpdateCell(best_pose, laser_scan_);
  // }

  // std::cout << "UpdateMap: updated " << particles_.size() << " particles"
  //           << std::endl;
}

void Gmapping::ComputeAndNormalizeWeights() {
  double total_weight = 0.0;
  // std::cout << "computeAndNormalizeWeights: starting" << std::endl;
  // 遍历每个粒子计算权重
  // 多线程计算粒子权重
  for (size_t i = 0; i < particles_.size(); ++i) {
    double weight = ComputeParticleWeight(particles_[i]);
    particles_[i].SetWeight(weight);
  }
  // auto compute_weights = [this](size_t start, size_t end) {
  //   for (size_t i = start; i < end; ++i) {
  //     double weight = ComputeParticleWeight(particles_[i]);
  //     particles_[i].SetWeight(weight);
  //   }
  // };
  // std::thread t1(compute_weights, 0, particles_.size() / 2);
  // std::thread t2(compute_weights, particles_.size() / 2, particles_.size());
  // t1.join();
  // t2.join();
  for (const auto& particle : particles_) {
    total_weight += particle.GetWeight();
  }
  // std::cout << "computeWeights: completed" << std::endl;
  // 归一化权重
  if (total_weight > 0) {
    for (auto& particle : particles_) {
      double normalized = particle.GetWeight() / total_weight;
      particle.SetWeight(normalized);
    }
  }
  // std::cout << "NormalizeWeights: completed" << std::endl;
  // 计算N_nerf
  float nerf = 0.0;
  for (auto& particle : particles_) {
    nerf += particle.GetWeight() * particle.GetWeight();
  }
  nerf = 1 / nerf;
  if (nerf < nerf_threshold_) {
    Resample();
  }
  // std::cout << "Nerf: completed" << std::endl;
}

float Gmapping::ComputeParticleWeight(Particle& particle) {
  // std::cout << "ComputeParticleWeight: starting" << std::endl;
  GridMap& grid_map = particle.GetGridMap(); // 使用引用避免拷贝
  Pose2D p_pose = particle.GetPose();
  // std::cout << "ComputeParticleWeight: grid_map.Raycast: starting" <<
  // std::endl;
  float sigma = 0.05;
  float log_weight = 0;
  for (size_t i = 0; i < laser_scan_.ranges.size(); i++) {
    float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
    float global_angle = p_pose.GetPhi() + angle;
    // std::cout << "raycast" << std::endl;
    float distance = grid_map.Raycast(p_pose.x, p_pose.y, global_angle,
                                      laser_scan_.range_max);
    // std::cout << "rayout over" << std::endl;
    float diff = distance - laser_scan_.ranges[i];
    log_weight += -0.5 * (diff * diff) / (sigma * sigma);
  }
  // std::cout << "ComputeParticleWeight grid_map.Raycast: completed" <<
  // std::endl;
  return std::exp(log_weight);
}

void Gmapping::OptimizePose() {
  // 检查当前激光扫描点云是否为空
  if (laser_scan_.ranges.empty()) {
    std::cout << "Warning: Current laser scan points are empty, skipping ICP"
              << std::endl;
    return;
  }
  for (size_t i = 0; i < particles_.size(); ++i) {
    Particle& p = particles_[i];
    Pose2D optimized_pose = scan_matcher_.SearchMatch(p, laser_scan_);
    p.SetPose(optimized_pose); // 更新粒子位姿
    p.PerturbPose();
  }
  // auto process_particle_range = [this](size_t start, size_t end) {
  //   for (size_t i = start; i < end; ++i) {
  //     Particle& p = particles_[i];
  //     std::cout << " optimize3 " << i << std::endl;
  //     Pose2D optimized_pose = scan_matcher_.SearchMatch(p, laser_scan_);
  //     std::cout << "searchMatch " << i << std::endl;
  //     p.SetPose(optimized_pose); // 更新粒子位姿
  //     p.PerturbPose();
  //   }
  // };
  // std::cout << " optimize" << std::endl;
  // std::thread thread1(process_particle_range, 0, particles_.size() / 2);
  // std::thread thread2(process_particle_range, particles_.size() / 2,
  //                     particles_.size());
  // std::cout << " optimize2" << std::endl;
  // // 等待线程完成
  // thread1.join();
  // thread2.join();
  // RCLCPP_INFO(rclcpp::get_logger("gmapping"), "ICP took %ld ms",
  // duration2); p.SetPose(optimized_pose); p.PertubPose();
}

Pose2D Gmapping::GetBestEstimate() {
  // 返回权重最大的粒子位姿
  if (particles_.empty()) {
    // 如果没有粒子，返回默认位姿
    return Pose2D(0.0, 0.0, 0.0);
  }

  double max_weight = 0.0;
  Pose2D best_pose = particles_[0].GetPose(); // 初始化为第一个粒子

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

bool Gmapping::CheckReachEdge(Pose2D& pose) {
  for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
    float range = laser_scan_.ranges[i];
    if (range < laser_scan_.range_min || range > laser_scan_.range_max) {
      continue; // 跳过无效的激光点
    }
    // 计算激光点的全局坐标
    float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
    float global_x = pose.x + range * cos(pose.GetPhi() + angle);
    float global_y = pose.y + range * sin(pose.GetPhi() + angle);

    // 检查是否超出地图边界
    if (!grid_map_.isGridInBounds(grid_map_.x2idx(global_x),
                                  grid_map_.y2idy(global_y))) {
      return true; // 超出边界，需扩展地图
    }
  }
  return false; // 所有点都在边界内
}