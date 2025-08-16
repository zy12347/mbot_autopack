#include "mbot_interface/KDTree.h"

void KDTree::DestroyTree(KDTreeNode* node) {
  if (node == nullptr)
    return;
  DestroyTree(node->left);
  DestroyTree(node->right);
  delete node;
}

KDTreeNode* KDTree::BuildTree(std::vector<std::vector<float>>& points,
                              int depth) {
  if (points.empty())
    return nullptr;

  int axis = depth % k;
  std::sort(points.begin(), points.end(),
            [axis](const std::vector<float>& a, const std::vector<float>& b) {
              return a[axis] < b[axis];
            });

  int median = points.size() / 2;
  KDTreeNode* node = new KDTreeNode(points[median], axis);

  std::vector<std::vector<float>> left_points(points.begin(),
                                              points.begin() + median);
  std::vector<std::vector<float>> right_points(points.begin() + median + 1,
                                               points.end());

  node->left = BuildTree(left_points, depth + 1);
  node->right = BuildTree(right_points, depth + 1);

  return node;
}

void KDTree::NearestNeighbor(KDTreeNode* node, const std::vector<float>& query,
                             KDTreeNode*& best_node, float& best_dis_sq) {
  if (node == nullptr) {
    return;
  }
  float dis = GetDistanceSq(query, node->point);
  if (dis < best_dis_sq) {
    best_dis_sq = dis;
    best_node = node;
  }
  int axis = node->split_dim;
  KDTreeNode* first = nullptr;
  KDTreeNode* second = nullptr;
  if (query[axis] < node->point[axis]) {
    first = node->left;   // 优先搜索左子树
    second = node->right; // 可能需要回溯的子树
  } else {
    first = node->right; // 优先搜索右子树
    second = node->left; // 可能需要回溯的子树
  }
  NearestNeighbor(first, query, best_node, best_dis_sq);
  float plane_dist_sq =
      (query[node->split_dim] - node->point[node->split_dim]) *
      (query[node->split_dim] - node->point[node->split_dim]);
  if (plane_dist_sq < best_dis_sq) { // 距离超平面的距离 < 当前最佳距离
    NearestNeighbor(second, query, best_node,
                    best_dis_sq); // 回溯搜索另一子树
  }
}

std::vector<float> KDTree::FindNearestNeighbor(
    const std::vector<float>& query) {
  if (root == nullptr || int(query.size()) != k) {
    return std::vector<float>(); // 返回空向量表示树为空
  }

  KDTreeNode* best_node = root;
  float best_dist_sq = GetDistanceSq(root->point, query);
  NearestNeighbor(root, query, best_node, best_dist_sq);
  return best_node->point;
}
