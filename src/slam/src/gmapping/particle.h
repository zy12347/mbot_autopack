#pragma once
#include "mbot_common.h"

class Particle {
 public:
  Particle(Pose2D& init_pose) : pose(init_pose) { map = GridMap(); };
  void SetWeight(double w) { weight = w; };
  Pose2D GetPose() { return pose; };
  std::vector<std::pair<float, float>> ScanMap(float max_range);
  std::pair<int,int> Bresnham(GridMap& map,int start_x,int start_y,int end_x,int end_y);

 private:
  GridMap map;
  double weight;
  Pose2D pose;
  Pose2D init_pose;
};