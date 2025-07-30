class GridMap{
 public:
  GridMap(int w = 100, int h = 100, float r = 0.05)
      : width(w), height(h), resolution(r) {
    map_ = new uint8_t[width * height]();
  };
  ~GridMap() {
    delete[] map_;
    map_ = nullptr;
  }
  int GetWidth(){return width;};
  int GetHeight(){return height;};
  float GetResolution(){return resolution;};
  int x2idx(float x);
  int y2idy(float y);

  float idx2x(int x);
  float idy2y(int y);

  int GetValue(int idx,int idy);
  
  void UpdateCell();
  void GetDistance();
  void ToOccupancyGrid();

  void FromRosMsg(const mbot_interface::msg::GridMap);

  mbot_interface::msg::GridMap toRosMsg() const;

 private:
  int width = 100;
  int height = 100;
  float resolution = 0.05f;
  uint8_t* map_ = nullptr;
};