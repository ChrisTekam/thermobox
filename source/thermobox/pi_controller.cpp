#pragma once
#include <Arduino.h>

// Discrete PI controller. Output is a duty cycle limited to [outMin, outMax].
// The integrator uses conditional integration (clamping): it holds while the
// output is saturated and the error would push it further.
class PIController {
public:
  // Ts: sample time in seconds.
  PIController(float kp, float ki, float Ts)
    : kp_(kp), ki_(ki), Ts_(Ts),
      outMin_(0.0f), outMax_(1.0f),
      integral_(0.0f) {}

  void begin() { reset(); }

  void reset() { integral_ = 0.0f; }

  void setGains(float kp, float ki) {
    kp_ = kp;
    ki_ = ki;
  }

  void setOutputLimits(float outMin, float outMax) {
    outMin_ = outMin;
    outMax_ = outMax;
  }

  float compute(float setpoint, float measured) {
    float error = setpoint - measured;
    float P = kp_ * error;

    float uUnsat = P + integral_;
    bool pushingHigh = (uUnsat >= outMax_) && (error > 0.0f);
    bool pushingLow  = (uUnsat <= outMin_) && (error < 0.0f);
    if (!pushingHigh && !pushingLow) {
      integral_ += ki_ * error * Ts_;
    }
    integral_ = constrain(integral_, outMin_, outMax_);

    float u = P + integral_;
    return constrain(u, outMin_, outMax_);
  }

private:
  float kp_, ki_;
  float Ts_;
  float outMin_, outMax_;
  float integral_;
};
