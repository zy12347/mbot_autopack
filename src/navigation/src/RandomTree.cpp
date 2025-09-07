#include "RandomTree.h"

std::vector<std::pair<int, int>> RandomTree::PlanPath(
    int start_idx, int start_idy, int target_idx, int target_idy, int step_size,
    int max_iterations) {
  if (map_data_ == nullptr || width_ <= 0 || height_ <= 0 ||
      IsValid(start_idx, start_idy) == false ||
      IsValid(target_idx, target_idy) == false) {
    std::cerr << "Map data is null!" << std::endl;
    return std::vector<std::pair<int, int>>();
  }
  tree_.clear();
  Node root(start_idx, start_idy, -1);
  tree_.emplace_back(root);
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution_x(0, width_ - 1);
  std::uniform_int_distribution<int> distribution_y(0, height_ - 1);

  for (int iter = 0; iter < max_iterations; ++iter) {
    int rand_x = distribution_x(generator);
    int rand_y = distribution_y(generator);
    if (!IsValid(rand_x, rand_y)) {
      continue;
    }
    // 找到树中距离随机点最近的节点
    auto nearest_it = std::min_element(
        tree_.begin(), tree_.end(),
        [rand_x, rand_y](const Node& a, const Node& b) {
          float dist_a = std::hypot(a.x - rand_x, a.y - rand_y);
          float dist_b = std::hypot(b.x - rand_x, b.y - rand_y);
          return dist_a < dist_b;
        });
    if (nearest_it == tree_.end()) {
      std::cerr << "No nearest node found!" << std::endl;
      continue;
    }
    int new_x = step_size * (rand_x - nearest_it->x) /
                    std::hypot(rand_x - nearest_it->x, rand_y - nearest_it->y) +
                nearest_it->x;
    int new_y = step_size * (rand_y - nearest_it->y) /
                    std::hypot(rand_x - nearest_it->x, rand_y - nearest_it->y) +
                nearest_it->y;
    if (IsPathPassable(nearest_it->x, nearest_it->y, new_x, new_y)) {
      Node new_node(new_x, new_y, std::distance(tree_.begin(), nearest_it));
      tree_.emplace_back(new_node);
      if (IsTarget(new_x, new_y, target_idx, target_idy)) {
        std::cout << "Target reached in " << iter + 1 << " iterations."
                  << std::endl;
        return BuildPath(new_node);
      }
    }
  }
  return std::vector<std::pair<int, int>>();
}

std::vector<std::pair<int, int>> RandomTree::BuildPath(Node& tree) {
  std::vector<std::pair<int, int>> path;
  int current_idx = tree.parent_idx;
  path.emplace_back(tree.x, tree.y);
  while (current_idx != -1) {
    Node& current_node = tree_[current_idx];
    path.emplace_back(current_node.x, current_node.y);
    current_idx = current_node.parent_idx;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool RandomTree::IsPathPassable(int x1, int y1, int x2, int y2) {
  // Bresenham算法核心逻辑
  int dx = std::abs(x2 - x1);
  int dy = std::abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;

  int current_x = x1;
  int current_y = y1;

  while (true) {
    // 检查栅格是否在地图范围内
    if (!IsValid(current_x, current_y)) {
      return false;
    }
    // 到达终点，退出循环
    if (current_x == x2 && current_y == y2) {
      break;
    }

    // 更新下一个栅格
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      current_x += sx;
    }
    if (e2 < dx) {
      err += dx;
      current_y += sy;
    }
  }
  return true;
}

void RandomTree::PlotMap(std::vector<std::pair<int, int>>& path) {
  if (path.empty())
    return;
  cv::Mat img(height_, width_, CV_8UC1);
  memcpy(img.data, map_data_, height_ * width_ * sizeof(uint8_t));
  //   cv::Mat img(height_, width_, CV_8UC1, map_data_);
  for (auto& p : path) {
    img.at<uint8_t>(p.second, p.first) = 127;
  }
  cv::imshow("img", img);
  cv::waitKey(0);
  cv::imwrite("rrt_path.png", img);
}

void RandomTree::PlotTree() {
  cv::Mat img(height_, width_, CV_8UC1);
  memcpy(img.data, map_data_, height_ * width_ * sizeof(uint8_t));
  //   cv::Mat img(height_, width_, CV_8UC1, map_data_);
  for (auto& node : tree_) {
    if (node.parent_idx != -1) {
      Node& parent_node = tree_[node.parent_idx];
      cv::line(img, cv::Point(node.x, node.y),
               cv::Point(parent_node.x, parent_node.y), cv::Scalar(127), 1);
    }
  }
  cv::imshow("img", img);
  cv::waitKey(0);
  cv::imwrite("rrt_tree.png", img);
}