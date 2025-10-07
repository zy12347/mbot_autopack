#include "Astar.h"
std::vector<std::pair<int, int>> Astar::PlanPath(int start_idx, int start_idy,
                                                 int target_idx,
                                                 int target_idy) {
  if (map_data_ == nullptr) {
    std::cout << "map_data nullptr" << std::endl;
    return std::vector<std::pair<int, int>>();
  }
  if (start_idx < 0 || start_idx >= width_ || start_idy < 0 ||
      start_idy >= height_ || target_idx < 0 || target_idx >= width_ ||
      target_idy < 0 || target_idy >= height_) {
    std::cout << "unvalid id" << std::endl;
    return std::vector<std::pair<int, int>>();
  }
  target_idx_ = target_idx;
  target_idy_ = target_idy;
  std::cout << "start: " << start_idx << "," << start_idy
            << ", target: " << target_idx_ << "," << target_idy_ << std::endl;
  open_map_[start_idy * width_ + start_idx] = std::make_pair(0, -1);
  while (!open_map_.empty()) {
    int cur_node_index = GetMinScoreIndex();
    // std::cout << "cur_node_index: " << cur_node_index % width_ << ","
    //           << cur_node_index / width_ << std::endl;
    ProcessNeighbor(cur_node_index % width_, cur_node_index / width_);
    if (IsTarget(cur_node_index % width_, cur_node_index / width_)) {
      std::cout << "Found path! cur_node_index: " << cur_node_index % width_
                << " " << cur_node_index / width_ << std::endl;
      return BuildPath(cur_node_index % width_, cur_node_index / width_);
    }
    // for (auto it = open_map_.begin(); it != open_map_.end(); it++) {
    //   std::cout << it->first % width_ << "," << it->first / width_ << ":"
    //             << it->second.first << std::endl;
    // }
    // for (auto it = close_map_.begin(); it != close_map_.end(); it++) {
    //   std::cout << it->first % width_ << "," << it->first / width_ << ":"
    //             << it->second << std::endl;
    // }
  }
  std::cout << "Failed to find path!" << std::endl;
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
  return min_node_index;
}

float Astar::HeuristicScore(int x, int y) {
  return std::abs(x - target_idx_) + std::abs(y - target_idy_);
}
void Astar::ProcessNeighbor(int x, int y) {
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (!IsValid(x + dx, y + dy) || (dx == 0 && dy == 0))
        continue;
      float base_score =
          open_map_[y * width_ + x].first + std::sqrt(dx * dx + dy * dy);
      float heuristic_score = HeuristicScore(x + dx, y + dy);
      float total_score = base_score + heuristic_score;
      int index = (y + dy) * width_ + (x + dx);
      if (open_map_.find(index) == open_map_.end()) {
        open_map_[index] = std::make_pair(total_score, y * width_ + x);
      } else if (open_map_[index].first > total_score) {
        open_map_[index] = std::make_pair(total_score, y * width_ + x);
      }
    }
  }
  close_map_[y * width_ + x] = open_map_[y * width_ + x].second;
  open_map_.erase(y * width_ + x);
}
std::vector<std::pair<int, int>> Astar::BuildPath(int x, int y) {
  std::vector<std::pair<int, int>> path;
  int current_index = y * width_ + x;
  // std::cout << "current index " << current_index << std::endl;
  // for (auto it = close_map_.begin(); it != close_map_.end(); it++) {
  //   std::cout << it->first % width_ << "," << it->first / width_ << ":"
  //             << it->second << std::endl;
  // }
  while (current_index != -1) {
    path.emplace_back(current_index % width_, current_index / width_);
    current_index = close_map_[current_index];
    // std::cout << "current_index: " << current_index % width_ << ","
    //           << current_index / width_ << std::endl;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void Astar::PrintPath(std::vector<std::pair<int, int>>& path) {
  for (auto& p : path) {
    std::cout << p.first << " " << p.second << std::endl;
  }
}

void Astar::PlotMap(std::vector<std::pair<int, int>>& path) {
  cv::Mat img(height_, width_, CV_8UC1, map_data_);
  for (auto& p : path) {
    img.at<uint8_t>(p.second, p.first) = 127;
  }
  cv::imshow("img", img);
  cv::waitKey(0);
  cv::imwrite("astar_path.png", img);
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