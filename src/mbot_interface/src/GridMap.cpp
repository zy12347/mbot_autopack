#include "mbot_interface/GridMap.h"

#include <algorithm>
#include <cmath>

void GridMap::FromRosMsg(const mbot_interface::msg::GridMap& msg) {
  // 如果尺寸和分辨率一致，则直接复制数据
  if (width == msg.width && height == msg.height &&
      resolution == msg.resolution && map_ != nullptr) {
    if (msg.data.size() == width * height) {
      for (size_t i = 0; i < msg.data.size(); ++i) {
        map_[i] = static_cast<uint8_t>(msg.data[i]);
      }
    }
  } else {
    // 更新栅格图的尺寸和分辨率
    width = msg.width;
    height = msg.height;
    resolution = msg.resolution;

    // 重新分配内存
    if (map_ != nullptr) {
      delete[] map_;
    }
    map_ = new uint8_t[width * height]();

    // 复制栅格数据
    if (msg.data.size() == width * height) {
      for (size_t i = 0; i < msg.data.size(); ++i) {
        map_[i] = static_cast<uint8_t>(msg.data[i]);
      }
    }
  }
}

mbot_interface::msg::GridMap GridMap::ToRosMsg() const {
  mbot_interface::msg::GridMap msg;
  msg.width = width;
  msg.height = height;
  msg.resolution = resolution;
  msg.data.resize(width * height);
  for (int i = 0; i < width * height; ++i) {
    msg.data[i] = static_cast<uint8_t>(map_[i]);
  }
  return msg;
}

//局部区域提取,直接遍历不适用bresnham算法
std::vector<std::pair<float, float>> GridMap::ScanMap(const Pose2D& pose,
                                                      float max_range) const {
  std::vector<std::pair<float, float>> points;
  int min_grid_x = std::max(0, x2idx(pose.x - max_range));
  int max_grid_x = std::min(width, x2idx(pose.x + max_range));
  int min_grid_y = std::max(0, y2idy(pose.y - max_range));
  int max_grid_y = std::min(height, y2idy(pose.y + max_range));

  for (int x = min_grid_x; x <= max_grid_x; x++) {
    for (int y = min_grid_y; y <= max_grid_y; y++) {
      if (GetValue(x, y)) {
        points.push_back(std::make_pair(idx2x(x), idy2y(y)));
      }
    }
  }

  return points;
}

std::vector<std::pair<int, int>> GridMap::Bresnham(int start_idx, int start_idy,
                                                   int end_idx, int end_idy,
                                                   float max_range) const {
  std::vector<std::pair<int, int>> grid_path;

  // Bresenham算法核心逻辑
  int dx = std::abs(end_idx - start_idx);
  int dy = std::abs(end_idy - start_idy);
  int sx = (start_idx < end_idx) ? 1 : -1;
  int sy = (start_idy < end_idy) ? 1 : -1;
  int err = dx - dy;

  int current_idx = start_idx;
  int current_idy = start_idy;

  while (true) {
    // 检查栅格是否在地图范围内
    if (isGridInBounds(current_idx, current_idy)) {
      grid_path.emplace_back(current_idx, current_idy);
    }

    // 到达终点，退出循环
    if (current_idx == end_idx && current_idy == end_idy) {
      break;
    }

    // 更新下一个栅格
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      current_idx += sx;
    }
    if (e2 < dx) {
      err += dx;
      current_idy += sy;
    }
  }

  return grid_path;
}

float GridMap::Raycast(double start_x, double start_y, double angle,
                       double max_range) const {
  int start_idx = x2idx(start_x);
  int start_idy = y2idy(start_y);
  // 计算射线终点（最大射程处）
  double end_x = start_x + max_range * std::cos(angle);
  double end_y = start_y + max_range * std::sin(angle);

  // 获取射线经过的所有栅格
  int end_idx = x2idx(end_x);
  int end_idy = y2idy(end_y);
  auto grid_path = Bresnham(start_idx, start_idy, end_idx, end_idy, max_range);

  // 遍历栅格，寻找第一个障碍物
  for (const auto& [gx, gy] : grid_path) {
    if (GetValue(gx, gy) == 1) {
      // 计算障碍物距离起点的物理距离
      float wx = idx2x(gx);
      float wy = idy2y(gy);
      float distance = std::hypot(wx - start_x, wy - start_y);
      return distance;  // 击中障碍物
    }
  }

  // 未击中障碍物，返回最大射程
  return max_range;
}

void GridMap::UpdateCell(const Pose2D& pose, const LaserScan& scan) {
  for (size_t k = 0; k < scan.ranges.size(); ++k) {
    double range = scan.ranges[k];
    // 过滤无效数据
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }

    // 1. 计算激光束在世界坐标系中的终点（障碍物位置）
    double angle = scan.angle_min + k * scan.angle_increment;
    double end_x = pose.x + range * std::cos(angle + pose.GetPhi());
    double end_y = pose.y + range * std::sin(angle + pose.GetPhi());

    int start_idx = x2idx(pose.x);
    int start_idy = y2idy(pose.y);
    int end_idx = x2idx(end_x);
    int end_idy = y2idy(end_y);

    // 2. 射线投射：获取从机器人到障碍物的所有栅格
    auto grid_path =
        Bresnham(start_idx, start_idy, end_idx, end_idy, scan.range_max);

    // 3. 更新栅格：路径上的栅格标记为空闲，终点标记为占据
    if (!grid_path.empty()) {
      // 标记终点栅格为占据（最后一个栅格）
      auto [end_gx, end_gy] = grid_path.back();
      SetValue(end_gx, end_gy, 1);  // true表示占据

      // 标记路径上的其他栅格为空闲（除了终点）
      for (size_t i = 0; i < grid_path.size() - 1; ++i) {
        auto [gx, gy] = grid_path[i];
        SetValue(gx, gy, 0);  // false表示空闲
      }
    }
  }
}