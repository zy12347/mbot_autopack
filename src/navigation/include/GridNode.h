class GridNode {
  GridNode(int x, int y) : x(x), y(y){};
  GridNode(int x, int y, GridNode* parent) : x(x), y(y), parent(parent){};
  int x;
  int y;
  GridNode* parent = nullptr;
};