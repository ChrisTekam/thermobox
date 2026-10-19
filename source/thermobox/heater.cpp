#pragma once
#include <Arduino.h>

// Drives the heater through the inverting 2N3904 stage: pin LOW = heater on.
class Heater {
public:
  explicit Heater(uint8_t pin) : pin_(pin) {}

  // Call first in setup() so the heater starts off.
  void begin() {
    pinMode(pin_, OUTPUT);
    off();
  }

  // Both functions return the value written with analogWrite().
  uint8_t off() {
    analogWrite(pin_, 255);
    return 255;
  }

  // duty: 0.0 = off, 1.0 = full power.
  uint8_t setDuty(float duty) {
    if (isnan(duty)) return off();
    duty = constrain(duty, 0.0f, 1.0f);
    uint8_t pwm = 255 - (uint8_t)(duty * 255.0f + 0.5f);
    analogWrite(pin_, pwm);
    return pwm;
  }

private:
  uint8_t pin_;
};
