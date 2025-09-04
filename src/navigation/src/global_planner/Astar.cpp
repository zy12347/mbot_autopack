#include "Astar.h"
std::vector<std::pair<int, int>> Astar::PlanPath(int start_idx, int start_idy,
                                                 int target_idx,
                                                 int target_idy) {
  if (map_data_ == nullptr) {
    return std::vector<std::pair<int, int>>();
  }
  target_idx_ = target_idx;
  target_idy_ = target_idy;
  open_map_[start_idy * width_ + start_idx] = std::make_pair(0, -1);
  while (!open_map_.empty()) {
    int cur_node_index = GetMinScoreIndex();
    if (IsTarget(cur_node_index % width_, cur_node_index / width_)) {
      return BuildPath(cur_node_index % width_, cur_node_index / width_);
    }
    ProcessNeighbor(cur_node_index % width_, cur_node_index / width_);
  }
  return std::vector<std::pair<int, int>>();
}

int Astar::GetMinScoreIndex() {
  float min_score = std::numeric_limits<float>::max();
  int min_node_index = -1;
  for (auto it = open_map_.begin(); it != open_map_.end(); it++) {
    if (it->second.first < min_score) {
      min_score = it->second.first;
      min_node_index = it->first;
    }
  }
  open_map_.erase(min_node_index);
  return min_node_index;
}
void Astar::ProcessNeighbor(int x,int y) {
  close_map_[y * width_ + x] = open_map_[y * width_ + x].second;
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (!IsValid(x + dx, y + dy))
        continue;
      float base_score = open_map_[y * width_ + x].first + 1;
      float heuristic_score = HeuristicScore(x + dx, y + dy);
      float total_score = base_score + heuristic_score;
      int index = (y + dy) * width_ + (x + dx);
      if(open_map_.find(index) == open_map_.end()){
        open_map_[index] = std::make_pair(total_score, y * width_ + x);
      }else if(open_map_[index].first > total_score){
        open_map_[index] = std::make_pair(total_score, y * width_ + x);
      }
    }
  }
}
std::vector<std::pair<int, int>> BuildPath(int x,int y) {
  std::vector<std::pair<int, int>> path;
  int current_index = y * width_ + x;
  while(current_index != -1){
    path.emplace_back(current_index % width_, current_index / width_);
    current_index = close_map_[current_index];
  }
  return std::reverse(path.begin(), path.end());
}

// void Astar::ProcessNode(GridNode& neighbor_node) {
//   float total_score = TotalScore(neighbor_node);
//   if(open_map_.find(index) == open_map_.end()){
//     open_map_[index] = total_score;
//   }else if(open_map_[index] > total_score){
//     open_map_[index] = total_score;
//   }
// }

// float Astar::BaseScore(GridNode& node) {
//   return node.parent->score + 1;
// }
// float Astar::HeuristicScore(int x, int y) {
//   return std::abs(x - target_idx_) + std::abs(y - target_idy_);
// }
// float Astar::TotalScore(GridNode& node) {
//   float base_score = BaseScore(node);
//   float heuristic_score = HeuristicScore(node);
//   float total_score = heuristic_score + base_score;
//   return total_score;
// }