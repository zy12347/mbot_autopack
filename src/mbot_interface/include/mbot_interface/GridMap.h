#pragma once
#include <algorithm>

#include <opencv2/opencv.hpp>
#include "mbot_interface/LaserScan.h"
#include "mbot_interface/Pose2D.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

class GridMap {
 public:
  GridMap(int w = 400, int h = 400, float r = 0.05)
      : width(w), height(h), resolution(r) {
    origin_x = -width * resolution / 2;
    origin_y = -height * resolution / 2;
    map_ = new uint8_t[width * height]();
    // std::cout << "GridMap: Constructor called, map_=" << (void*)map_
    //           << std::endl;
  };

  // 拷贝构造函数
  GridMap(const GridMap& other)
      : width(other.width), height(other.height), resolution(other.resolution) {
    map_ = new uint8_t[width * height];
    std::copy(other.map_, other.map_ + width * height, map_);
    // std::cout << "GridMap: Copy constructor called, map_=" << (void*)map_
    //           << std::endl;
  }

  // 赋值运算符
  GridMap& operator=(const GridMap& other) {
    if (this != &other) {
      // std::cout << "GridMap: Assignment operator called, old map_="
      //           << (void*)map_ << std::endl;
      delete[] map_;
      width = other.width;
      height = other.height;
      resolution = other.resolution;
      map_ = new uint8_t[width * height];
      std::copy(other.map_, other.map_ + width * height, map_);
      // std::cout << "GridMap: Assignment operator completed, new map_="
      //           << (void*)map_ << std::endl;
    }
    return *this;
  }

  ~GridMap() {
    // std::cout << "GridMap: Destructor called, map_=" << (void*)map_
    //           << std::endl;
    delete[] map_;
    map_ = nullptr;
  }
  uint8_t* GetMapData() {
    return map_;
  };

  float GetOriginX() const {
    return origin_x;
  };

  float GetOriginY() const {
    return origin_y;
  };

  int GetWidth() const {
    return width;
  };
  int GetHeight() const {
    return height;
  };
  float GetResolution() const {
    return resolution;
  };
  int x2idx(float x) const {
    return int((x - origin_x) / resolution);
  };
  int y2idy(float y) const {
    return int((y - origin_y) / resolution);
  };

  float idx2x(int x) const {
    return float(x * resolution + origin_x);
  };
  float idy2y(int y) const {
    return float(y * resolution + origin_y);
  };

  bool isGridInBounds(int idx, int idy) const {
    return idx >= 0 && idx < width && idy >= 0 && idy < height;
  };

  int GetValue(int idx, int idy) const {
    return map_[idy * width + idx];
  };

  void SetValue(int idx, int idy, uint8_t value) {
    map_[idy * width + idx] = value;
  };

  void UpdateCell(const Pose2D& pose, const LaserScan& scan);

  void FromRosMsg(const nav_msgs::msg::OccupancyGrid& msg);

  nav_msgs::msg::OccupancyGrid ToRosMsg() const;

  std::vector<std::pair<float, float>> ScanMap(const Pose2D& pose,
                                               float max_range);

  float Raycast(double start_x, double start_y, double angle, double max_range);

  std::vector<std::pair<int, int>> Bresnham(int start_idx, int start_idy,
                                            int end_idx, int end_idy) const;

  void SaveAsBmp(std::string filename);

  void ExtendMap();

  int ComputeScore(std::vector<std::pair<float, float>>& global_pts);

 private:
  int width = 400;
  int height = 400;
  float resolution = 0.05f;
  float origin_x = -width * resolution / 2;
  float origin_y = -height * resolution / 2;
  uint8_t* map_ = nullptr;
};