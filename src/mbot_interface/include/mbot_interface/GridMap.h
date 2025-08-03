#pragma once
#include "mbot_interface/LaserScan.h"
#include "mbot_interface/Pose2D.h"
#include "mbot_interface/msg/grid_map.hpp"

class GridMap {
 public:
  GridMap(int w = 100, int h = 100, float r = 0.05)
      : width(w), height(h), resolution(r) {
    map_ = new uint8_t[width * height]();
  };
  ~GridMap() {
    delete[] map_;
    map_ = nullptr;
  }
  int GetWidth() const { return width; };
  int GetHeight() const { return height; };
  float GetResolution() const { return resolution; };
  int x2idx(float x) const { return int(x / resolution); };
  int y2idy(float y) const { return int(y / resolution); };

  float idx2x(int x) const { return float(x * resolution); };
  float idy2y(int y) const { return float(y * resolution); };

  bool isGridInBounds(int idx, int idy) const {
    return idx >= 0 && idx < width && idy >= 0 && idy < height;
  };

  int GetValue(int idx, int idy) const { return map_[idy * width + idx]; };

  void SetValue(int idx, int idy, uint8_t value) {
    map_[idy * width + idx] = value;
  };

  void UpdateCell(const Pose2D& pose, const LaserScan& scan);

  void FromRosMsg(const mbot_interface::msg::GridMap& msg);

  mbot_interface::msg::GridMap ToRosMsg() const;

  std::vector<std::pair<float, float>> ScanMap(const Pose2D& pose,
                                               float max_range) const;

  float Raycast(double start_x, double start_y, double angle,
                double max_range) const;

  std::vector<std::pair<int, int>> Bresnham(int start_idx, int start_idy,
                                            int end_idx, int end_idy,
                                            float max_range) const;

 private:
  int width = 100;
  int height = 100;
  float resolution = 0.05f;
  uint8_t* map_ = nullptr;
};