#pragma once
#include <cmath>

#include "mbot_interface/msg/pose2_d.hpp"

class Pose2D {
 public:
  Pose2D(float x = 0.0, float y = 0.0, float phi = 0.0)
      : x(x), y(y), Phi_(Wrap2PI(phi)) {}

  static float Wrap2PI(float phi) {
    while (phi > M_PI)
      phi = phi - 2 * M_PI;
    while (phi < -M_PI)
      phi = phi + 2 * M_PI;
    return phi;
  }
  static float DEG2RAD(float theta) {
    return theta / 180.0 * M_PI;
  }

  static float RAD2DEG(float phi) {
    return phi / M_PI * 180.0;
  }

  float GetPhi() const {
    return Phi_;
  };

  float GetTheta() const {
    return Phi_ / M_PI * 180.0;
  };

  void SetPhi(float phi) {
    Phi_ = Wrap2PI(phi);
  };

  void FromRosMsg(const mbot_interface::msg::Pose2D& msg) {
    x = msg.x;
    y = msg.y;
    Phi_ = msg.phi;
  }

  mbot_interface::msg::Pose2D ToRosMsg() const {
    mbot_interface::msg::Pose2D msg;
    msg.x = x;
    msg.y = y;
    msg.phi = Phi_;
    return msg;
  }

  float x;
  float y;
  float Phi_;
};