#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

class RandomTree {
 public:
  struct Node {
    int x, y;
    int parent_idx;

    Node(int x, int y, int parent_idx) : x(x), y(y), parent_idx(parent_idx) {}
  };

  RandomTree(int width, int height, uint8_t* map_data)
      : width_(width), height_(height), map_data_(map_data) {
    size_ = width_ * height_;
  };
  void UpdateMap(uint8_t* map_data) {
    map_data_ = map_data;
  };

  std::vector<std::pair<int, int>> PlanPath(int start_idx, int start_idy,
                                            int target_idx, int target_idy,
                                            int step_size = 1,
                                            int max_iterations = 1000);
  void PrintPath(std::vector<std::pair<int, int>>& path);

  void PlotMap(std::vector<std::pair<int, int>>& path);

  void PlotTree();

 private:
  bool IsValid(int x, int y) {
    return x >= 0 && x < width_ && y >= 0 && y < height_ &&
           map_data_[y * width_ + x] > 0;
  }

  bool IsTarget(int x, int y, int target_x, int target_y, int threshold = 2) {
    return std::hypot(x - target_x, y - target_y) < threshold;
  }

  bool IsPathPassable(int x1, int y1, int x2, int y2);

  std::vector<std::pair<int, int>> BuildPath(Node& tree);
  int width_;
  int height_;
  int size_;
  int threshold_ = 2;
  uint8_t* map_data_ = nullptr;
  std::vector<Node> tree_;
};