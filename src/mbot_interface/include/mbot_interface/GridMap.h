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