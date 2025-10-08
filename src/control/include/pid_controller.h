#pragma once
#include <algorithm>
#include <chrono> // 需包含时间库
#include <iostream>

class PIDController {
 public:
  PIDController() {};
  PIDController(double kp, double ki, double kd, float min_output,
                float max_output)
      : kp_(kp),
        ki_(ki),
        kd_(kd),
        min_output_(min_output),
        max_output_(max_output) {};

  void reset() {
    last_error_ = 0.0;
    integral_ = 0.0;
    last_time_ = std::chrono::steady_clock::now();
  }

  void setParams(double kp, double ki, double kd, float min_output,
                 float max_output) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    min_output_ = min_output;
    max_output_ = max_output;
  }

  float compute(float error);

 private:
  float kp_;
  float ki_;
  float kd_;
  float min_output_;                                // 最小输出限制
  float max_output_;                                // 最大输出限制
  float last_error_;                                // 上一时刻误差
  float integral_;                                  // 积分累积
  std::chrono::steady_clock::time_point last_time_; // 上一时刻时间（用于计算T）
};