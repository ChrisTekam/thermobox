#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Non-blocking DS18B20 wrapper: startConversion() returns immediately and
// read() collects the result on the next tick. The loop period must be longer
// than conversionTimeMs().
class TempSensor {
public:
  // resolutionBits: 9 (0.5 C, ~94 ms) to 12 (0.0625 C, ~750 ms).
  TempSensor(uint8_t pin, uint8_t resolutionBits = 9)
    : oneWire_(pin),
      sensors_(&oneWire_),
      resBits_(constrain(resolutionBits, 9, 12)),
      lastTemp_(NAN),
      valid_(false),
      present_(false) {}

  // Returns false if no sensor is found.
  bool begin() {
    sensors_.begin();
    present_ = sensors_.getAddress(addr_, 0);
    if (!present_) return false;
    sensors_.setResolution(addr_, resBits_);
    sensors_.setWaitForConversion(false);
    return true;
  }

  void startConversion() {
    if (present_) sensors_.requestTemperatures();
  }

  // Returns the last good reading; isValid() tells whether this read was good.
  float read() {
    if (!present_) {
      valid_ = false;
      return lastTemp_;
    }

    float t = sensors_.getTempC(addr_);

    // 85.0 is the DS18B20 power-on value, returned if read before a conversion finishes.
    valid_ = (t != DEVICE_DISCONNECTED_C) &&
             (t >= kMinPlausibleC) && (t <= kMaxPlausibleC) &&
             (t != 85.0f);
    if (valid_) lastTemp_ = t;
    return lastTemp_;
  }

  bool isValid() const { return valid_; }

  uint16_t conversionTimeMs() const {
    switch (resBits_) {
      case 9:  return 94;
      case 10: return 188;
      case 11: return 375;
      default: return 750;
    }
  }

private:
  static constexpr float kMinPlausibleC = -10.0f;
  static constexpr float kMaxPlausibleC = 125.0f;

  OneWire           oneWire_;
  DallasTemperature sensors_;
  DeviceAddress     addr_;
  uint8_t           resBits_;
  float             lastTemp_;
  bool              valid_;
  bool              present_;
};
