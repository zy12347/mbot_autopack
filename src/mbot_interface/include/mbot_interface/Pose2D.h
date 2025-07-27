#pragma once
#include <cmath>

class Pose2D {
 public:
  Pose2D(float x = 0.0, float y = 0.0, float theta = 0.0)
      : x(x), y(y), phi(Wrap2PI(theta)) {}
  static float Wrap2PI(float theta) {
    while (theta > M_PI) theta = theta - 2 * M_PI;
    while (theta < M_PI) theta = theta + 2 * M_PI;
    return theta;
  }
  static float DEG2RAD(float theta) { return theta / 180.0 * M_PI; }

  static float RAD2DEG(float phi) { return phi / M_PI * 180.0; }

  float x;
  float y;
  float phi;
};