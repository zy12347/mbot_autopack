#include "Astar.h"
std::vector<std::pair<int, int>> Astar::PlanPath(int start_idx, int start_idy,
                                                 int target_idx,
                                                 int target_idy) {
  if (map_data_ == nullptr) {
    return;
  }
  GridNode base_node = GridNode(start_idx, start_idy, 0);
  next_list_.emplace(base_node);
  while (!next_list_.empty()) {
    GridNode cur_node = next_list_.top().node;
    next_list.pop();
    if (cur_node.x == targer_idx && cur_node.y == target_idy) {
      break;
    }
    ProcessNeighbor(cur_node);
  }
  BuildPath();
}

void Astar::ProcessNeighbor(GridNode& node) {
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0)
        continue;
      ProcessNode(node.x + dx, node.y + dy);
    }
  }
}

void Astar::ProcessNode(int x, int y) {
  if (x >= 0 && x < width_ && y >= 0 && y < height_ && map_data_[index] > 0) {
    int index = y * width_ + x;
    if (open_close_set_[index] == 0) {
      float total_score = Totalscore(x, y);
      GridWithScore cur_node_score(x, y, total_score);
      next_list_.push(cur_node_score);
      open_close_set_[index] = 1;
    }
  }
}

float Astar::BaseScore(int x, int y) {}
float Astar::HeuristicScore(int x, int y) {}
float Astar::TotalScore(int x, int y) {
  float base_score = BaseScore(x, y);
  float heuristic_score = HeuristicScore(x, y);
  float total_score = heuristic_score + base_score;
  return total_score;
}