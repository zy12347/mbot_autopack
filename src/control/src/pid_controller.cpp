#include "pid_controller.h"

float PIDController::compute(float error) {
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<float> dt = now - last_time_; // 时间差
  float T = dt.count();
  last_time_ = now; // 更新上次时间
  std::cout << "T " << T << " error " << error << "integral " << integral_
            << std::endl;
  if (T <= 0.0f) { // 避免除零（如连续调用）
    return 0.0f;
  }
  if (fabs(error) < integral_threshold_) {
    integral_ += error * T; // 积分分离
  }
  integral_ = std::clamp(integral_, min_output_ / ki_,
                         max_output_ / ki_); // 积分限幅避免过度积累
  float output =
      kp_ * error + ki_ * integral_ + kd_ * (error - last_error_) / T;
  last_error_ = error;
  return std::clamp(output, min_output_, max_output_);
}