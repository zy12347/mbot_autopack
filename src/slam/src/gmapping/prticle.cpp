#include "particle.h"

//局部区域提取,直接遍历不适用bresnham算法
std::vector<std::pair<float, float>> Particle::ScanMap(float max_range) {
  std::vector<std::pair<float, float>> points;
  int min_grid_x = std::max(0, map.x2idx(init_pose.x - max_range));
  int max_grid_x = std::min(map.GetWidth(), map.x2idx(init_pose.x + max_range));
  int min_grid_y = std::max(0, map.y2idy(init_pose.y + max_range));
  int max_grid_y = std::min(map.GetHeight(), map.y2idy(init_pose.y + max_range));

  for(int x = min_grid_x; x <= max_grid_x; x++){
    for(int y = min_grid_y; y <= max_grid_y; y++){
      if(map.GetValue(x, y)){
        points.push_back(std::make_pair(map.idx2x(x), map.idy2y(y)));
      }
    }
  }

  return points;
}

std::pair<int, int> Particle::Bresnham(GridMap& map, int start_x, int start_y,
                                       int end_x, int end_y) {
  bool is_hit = false;
  int dx_grid = abs(end_idx - start_idx);
  int dy_grid = abs(end_idy - start_idy);

  int err = dx_grid - dy_grid;

  int step_x = end_idx > start_idx ? 1 : -1;
  int step_y = end_idy > start_idy ? 1 : -1;

  int current_idx = start_idx;
  int current_idy = start_idy;
  while (true) {
    float current_x = map.idx2x(current_idx);
    float current_y = map.idy2y(current_idy);
    float dist =
        (current_x - init_pose.x) * (current_x - init_pose.x) +
        (current_y - init_pose.y) *
            (current_y - init_pose.y) if (dist > max_range * max_range) {
      break;
    }

    if (map.GetValue(current_idx, current_idy)) {
      is_hit = true;
      point.first = map.idx2x(current_idx);
      point.second = map.idy2y(current_idy);
      break;
    }
    if (current_idx == end_idx && current_idy == end_idy) {
      break;
    }
    int err2 = 2 * err;
    if (err2 > -dy_grid) {
      err -= dy_grid;
      current_idx += step_x;
    }
    if (err2 < dx_grid) {
      err += dx_grid;
      current_idy += step_y;
    }
  }
  if (is_hit) {
    return std::make_pair(current_idx, current_idy);
  }
  return std::make_pair(0, 0);
}