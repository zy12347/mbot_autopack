#include "scanmatcher.h"

#include <cmath>
#include <iostream>

Pose2D ScanMatcher::ICP(std::vector<std::pair<float, float>>& p_s,
                        std::vector<std::pair<float, float>>& p_d,
                        Pose2D init_pose) {
  int max_iteration = 10;
  float rotation_epsilon = 1e-4;
  float translation_epsilon = 1e-4;

  if (p_d.empty() || p_s.empty()) {
    std::cout << "error: empty point clouds" << std::endl;
    return init_pose;
  }

  Eigen::Matrix3d T = GetTransForm(init_pose);
  std::vector<std::pair<float, float>> transformed_pts =
      TransFormPoints(p_s, T);
  Pose2D optimized_pose = init_pose;

  for (int i = 0; i < max_iteration; i++) {
    std::vector<std::pair<float, float>> correspondence_pts;
    FindCorresponcePoints(transformed_pts, p_d, correspondence_pts);

    if (correspondence_pts.empty()) {
      break;
    }

    Eigen::Matrix3d delta_T = ComputeOptimalTransform(correspondence_pts);
    T = delta_T * T;
    transformed_pts = TransFormPoints(p_s, T);

    float phi = atan2(T(1, 0), T(0, 0));
    float x = T(0, 2);
    float y = T(1, 2);

    float phi_err = std::abs(phi - optimized_pose.GetPhi());
    float x_err = std::abs(x - optimized_pose.x);
    float y_err = std::abs(y - optimized_pose.y);

    if (phi_err < rotation_epsilon && x_err < translation_epsilon &&
        y_err < translation_epsilon) {
      break;
    }
    optimized_pose = Pose2D(x, y, phi);
  }
  return optimized_pose;
}

// 该函数根据初始位姿生成对应的齐次变换矩阵（3x3）
Eigen::Matrix3d ScanMatcher::GetTransForm(Pose2D& init_pose) {
  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
  float phi = init_pose.GetPhi();
  float x = init_pose.x;
  float y = init_pose.y;
  T(0, 0) = cos(phi);
  T(0, 1) = -sin(phi);
  T(1, 0) = sin(phi);
  T(1, 1) = cos(phi);
  T(0, 2) = x;
  T(1, 2) = y;
  return T;
}

std::vector<std::pair<float, float>> ScanMatcher::TransFormPoints(
    std::vector<std::pair<float, float>>& points, Eigen::Matrix3d& T) {
  std::vector<std::pair<float, float>> transformed_points;
  for (auto& p : points) {
    Eigen::Vector3d p_homo(p.first, p.second, 1);
    Eigen::Vector3d transformed_p = T * p_homo;
    transformed_points.emplace_back(transformed_p(0), transformed_p(1));
  }
  return transformed_points;
}

void ScanMatcher::FindCorresponcePoints(
    std::vector<std::pair<float, float>>& transformed_pts,
    std::vector<std::pair<float, float>>& p_d,
    std::vector<std::pair<float, float>>& correspondence_pts) {
  correspondence_pts.clear();
  for (auto& p : transformed_pts) {
    float min_dist = 1000000;
    int min_index = -1;
    for (int i = 0; i < p_d.size(); i++) {
      float dist = sqrt(pow(p.first - p_d[i].first, 2) +
                        pow(p.second - p_d[i].second, 2));
      if (dist < min_dist) {
        min_dist = dist;
        min_index = i;
      }
    }
    if (min_index >= 0) {
      correspondence_pts.emplace_back(p.first, p.second);
      correspondence_pts.emplace_back(p_d[min_index].first,
                                      p_d[min_index].second);
    }
  }
}

Eigen::Matrix3d ScanMatcher::ComputeOptimalTransform(
    std::vector<std::pair<float, float>>& correspondence_pts) {
  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();

  if (correspondence_pts.size() < 4) {
    return T;  // 至少需要2个对应点对
  }

  // 计算质心
  double sum_x1 = 0, sum_y1 = 0, sum_x2 = 0, sum_y2 = 0;
  int num_pairs = correspondence_pts.size() / 2;

  for (int i = 0; i < num_pairs; i++) {
    sum_x1 += correspondence_pts[2 * i].first;
    sum_y1 += correspondence_pts[2 * i].second;
    sum_x2 += correspondence_pts[2 * i + 1].first;
    sum_y2 += correspondence_pts[2 * i + 1].second;
  }

  double center_x1 = sum_x1 / num_pairs;
  double center_y1 = sum_y1 / num_pairs;
  double center_x2 = sum_x2 / num_pairs;
  double center_y2 = sum_y2 / num_pairs;

  // 计算旋转矩阵
  double H11 = 0, H12 = 0, H21 = 0, H22 = 0;
  for (int i = 0; i < num_pairs; i++) {
    double dx1 = correspondence_pts[2 * i].first - center_x1;
    double dy1 = correspondence_pts[2 * i].second - center_y1;
    double dx2 = correspondence_pts[2 * i + 1].first - center_x2;
    double dy2 = correspondence_pts[2 * i + 1].second - center_y2;

    H11 += dx1 * dx2 + dy1 * dy2;
    H12 += dx1 * dy2 - dy1 * dx2;
    H21 += dy1 * dx2 - dx1 * dy2;
    H22 += dx1 * dx2 + dy1 * dy2;
  }

  // SVD分解求解旋转矩阵
  Eigen::Matrix2d H;
  H << H11, H12, H21, H22;
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

  // 构建变换矩阵
  T(0, 0) = R(0, 0);
  T(0, 1) = R(0, 1);
  T(1, 0) = R(1, 0);
  T(1, 1) = R(1, 1);
  T(0, 2) = center_x2 - R(0, 0) * center_x1 - R(0, 1) * center_y1;
  T(1, 2) = center_y2 - R(1, 0) * center_x1 - R(1, 1) * center_y1;

  return T;
}

float ScanMatcher::ComputeError(Pose2D& opt_pose, LaserScan& scan,
                                GridMap& grid_map) {
  float error = 0;
  return error;
}
