#include <algorithm>
#include <cmath>
#include <iostream>
// #include <priority_queue>
#include <limits>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

struct Node {
  int x, y;
  float base_score;
  float f_score;
  int parent;

  // 构造函数
  Node(int x, int y, float f_score, int parent)
      : x(x), y(y), f_score(f_score), parent(parent) {}

  // 按坐标确定唯一性
  bool operator==(const Node& other) const {
    return x == other.x && y == other.y;
  }
};

struct NodeComparator {
  bool operator()(const Node& a, const Node& b) const {
    if (a.f_score != b.f_score) {
      return a.f_score < b.f_score; // 按分数排序
    }
    // 分数相同时按坐标排序，确保唯一性
    if (a.x != b.x)
      return a.x < b.x;
    return a.y < b.y;
  }
};

class Astar {
 public:
  Astar(int width, int height, uint8_t* map_data)
      : width_(width), height_(height), map_data_(map_data) {
    size_ = width_ * height_;
  };
  void UpdateMap(uint8_t* map_data);

  std::vector<std::pair<int, int>> PlanPath(int start_idx, int start_idy,
                                            int target_idx, int target_idy);
  void PrintPath(std::vector<std::pair<int, int>>& path);

  void PlotMap(std::vector<std::pair<int, int>>& path);

 private:
  // float BaseScore(GridNode& node);
  // float HeuristicScore(GridNode& node);
  // float TotalScore(GridNode& node);
  float HeuristicScore(int x, int y);

  void ProcessNeighbor(int x, int y);
  bool IsValid(int x, int y) {
    return x >= 0 && x < width_ && y >= 0 && y < height_ &&
           map_data_ != nullptr && map_data_[y * width_ + x] > 0 &&
           close_map_.find(y * width_ + x) == close_map_.end();
  };
  bool IsTarget(int x, int y) {
    return x == target_idx_ && y == target_idy_;
  };
  int GetMinScoreIndex();
  std::vector<std::pair<int, int>> BuildPath(int x, int y);
  uint8_t* map_data_ = nullptr;
  int width_ = 100;
  int height_ = 100;
  int size_ = width_ * height_;

  int target_idx_ = 0;
  int target_idy_ = 0;

  std::unordered_map<int, std::pair<float, int>>
      open_map_; // 表示在open_list_中的节点的index及其total_score
  std::unordered_map<int, int>
      close_map_; // 表示在close_list_中的节点的index及其pre_index
};