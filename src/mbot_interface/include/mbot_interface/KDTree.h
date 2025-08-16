#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

struct KDTreeNode {
  KDTreeNode* left;
  KDTreeNode* right;
  std::vector<float> point;
  int split_dim;
  KDTreeNode(const std::vector<float>& p, int dim)
      : left(nullptr), right(nullptr), point(p), split_dim(dim) {}
};

class KDTree {
 private:
  KDTreeNode* root;
  int k; // dim

  KDTreeNode* BuildTree(std::vector<std::vector<float>>& points, int depth = 0);

  void NearestNeighbor(KDTreeNode* node, const std::vector<float>& points,
                       KDTreeNode*& best_node, float& best_dis_sq);

 public:
  KDTree(int k = 2) : root(nullptr), k(k) {}
  KDTree(std::vector<std::vector<float>>& points) {
    if (points.empty()) {
      root = nullptr;
      k = 0;
      return;
    }
    k = points[0].size();
    root = BuildTree(points, 0);
  }
  void DestroyTree(KDTreeNode* node);
  ~KDTree() {
    DestroyTree(root);
  }

  bool empty() {
    return root == nullptr;
  }

  KDTreeNode* GetKdTreeRoot() {
    return root;
  }

  float GetDistanceSq(const std::vector<float>& a,
                      const std::vector<float>& b) {
    float dist_sq = 0.0f;
    for (int i = 0; i < k; ++i) {
      float diff = a[i] - b[i];
      dist_sq += diff * diff;
    }
    return dist_sq;
  }
  std::vector<float> FindNearestNeighbor(const std::vector<float>& query);

  void BuildKdTree(std::vector<std::vector<float>>& points, int depth = 0) {
    if (points.empty()) {
      return;
    }
    if (root != nullptr) {
      DestroyTree(root);
    }
    k = points[0].size();
    root = BuildTree(points, depth);
  }
};