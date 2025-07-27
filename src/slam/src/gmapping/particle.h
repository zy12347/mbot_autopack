#pragma once
#include "mbot_common.h"

class Particle {
 public:
  Particle(Pose2D& init_pose) : pose(init_pose) { map = GridMap(); };
  void SetWeight(double w) { weight = w; };
  Pose2D GetPose() { return pose; };

 private:
  GridMap map;
  double weight;
  Pose2D pose;
};