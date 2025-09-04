#include <algorithm>
#include <cmath>
#include <iostream>
// #include <priority_queue>
#include <vector>
#include <unordered_map>

// struct GridNode{
//   GridNode(int x, int y) : x(x), y(y);
//   int x;

//   int y;
//   int base_score = 0;
//   int heuristic_score = 0;
//   int total_score = 0;

//   GridNode* parent = nullptr;
//   bool operator>(const GridNode& other) {
//     return total_score > other.total_score;
//   }
//   bool operator==(const GridNode& other) const {
//         return x == other.x && y == other.y;
//     }
// };

class Astar {
 public:
  Astar(int width, int height, uint8_t* map_data)
      : width_(width), height_(height), map_data_(map_data) {
    size_ = width_ * height_;
  };
  ~Astar();
  void UpdateMap(uint8_t* map_data);
  void ProcessNeighbor(int x,int y);
  std::vector<std::pair<int, int>> PlanPath(int start_idx, int start_idy,
                                            int target_idx, int target_idy);

 private:
  // float BaseScore(GridNode& node);
  // float HeuristicScore(GridNode& node);
  // float TotalScore(GridNode& node);

  bool IsValid(int x,int y){
    return x >= 0 && x < width_ && y >= 0 && y < height_ && map_data_[y * width_ + x] > 0 && close_map_.find(y * width_ + x) == close_map_.end();
  };
  bool IsTarget(int x,int y){
    return x == target_idx && y == target_idy;
  };
  int GetMinScoreIndex();
  std::vector<std::pair<int, int>> BuildPath(int x,int y);
  uint8_t* map_data_ = nullptr;
  int width_ = 100;
  int height_ = 100;
  int size_ = width_ * height_;

  int target_idx_ = 0;
  int target_idy_ = 0;

  std::unordered_map<int,std::pair<float,int>> open_map_;//表示在open_list_中的节点的index及其total_score
  std::unordered_map<int,int> close_map_;//表示在close_list_中的节点的index及其pre_index
};