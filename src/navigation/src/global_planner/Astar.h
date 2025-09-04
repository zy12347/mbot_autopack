#include <algorithm>
#include <cmath>
#include <iostream>
#include <priority_queue>
#include <vector>

#include "GridNode.h"
struct GridWithScore {
  GridWithScore(int x, int y, float score) : node(x, y), score(score);
  GridNode node;
  int score;
  bool operator(const GridWithScore& other) {
    return score > other.score;
  }
};

class Astar {
 public:
  Astar(int width, int height, uint8_t* map_data)
      : width_(width), height_(height), map_data_(map_data) {
    size_ = width_ * height_;
  };
  ~Astar();
  void UpdateMap(uint8_t* map_data);
  void ProcessNode(int x, int y);
  void ProcessNeighbor(GridNode& node);
  std::vector<std::pair<int, int>> PlanPath(int start_idx, int start_idy,
                                            int target_idx, int target_idy);

 private:
  float BaseScore(int x, int y);
  float HeuristicScore(int x, int y);
  float TotalScore(int x, int y);

  bool IsTarget(GridNode* node);
  std::vector<std::pair<int, int>> BuildPath();
  uint8_t* map_data_ = nullptr;
  int width_ = 100;
  int height_ = 100;
  int size_ = width_ * height_;

  std::priority_queue<GridWithScore> next_list_;
  GridNode* root;
  std::bitset<size_> open_close_set_;
}