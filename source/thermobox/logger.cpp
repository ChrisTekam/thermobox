#pragma once
#include <Arduino.h>

// Prints one CSV row per control tick: millis,setpoint,temperature,duty,pwm
// duty is the controller output (0.0-1.0); pwm is the value written to the heater pin.
class Logger {
public:
  explicit Logger(long baud = 9600) : baud_(baud) {}

  void begin() {
    Serial.begin(baud_);
    Serial.println(F("millis,setpoint,temperature,duty,pwm"));
  }

  void logRow(unsigned long timeMs, float setpoint, float temperature,
              float duty, uint8_t pwm) {
    Serial.print(timeMs);
    Serial.print(',');
    Serial.print(setpoint, 2);
    Serial.print(',');
    Serial.print(temperature, 2);
    Serial.print(',');
    Serial.print(duty, 3);
    Serial.print(',');
    Serial.println(pwm);
  }

private:
  long baud_;
};
