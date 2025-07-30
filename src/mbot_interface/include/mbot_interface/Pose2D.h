#pragma once
#include <cmath>

class Pose2D {
 public:
  Pose2D(float x = 0.0, float y = 0.0, float phi = 0.0)
      : x(x), y(y), Phi_(Wrap2PI(phi)) {}

  static float Wrap2PI(float phi) {
    while (phi > M_PI) phi = phi - 2 * M_PI;
    while (phi < M_PI) phi = phi + 2 * M_PI;
    return phi;
  }
  static float DEG2RAD(float theta) { return theta / 180.0 * M_PI; }

  static float RAD2DEG(float phi) { return phi / M_PI * 180.0; }

  float GetPhi(){return Phi_;};

  float GetTheta(){return Phi_ / M_PI * 180.0;};

  float x;
  float y;
  float Phi_;
};