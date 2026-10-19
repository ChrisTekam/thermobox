/*
  OPEN_LOOP_TEST.ino
  Open-loop step test of the thermal plant (no PID, no setpoint).

  On power-up / reset:
    1. 0 to BASELINE_S : heater off (logs ambient temperature)
    2. After BASELINE_S: heater on at STEP_DUTY_PCT, indefinitely
  There is no end: stop logging with Ctrl+C and switch off the supply.

  Serial output (9600 baud), matches log_serial.py:
    millis,setpoint,temperature,duty,pwm
      setpoint    : always "nan" (open loop)
      temperature : degC, or "nan" if the sensor read is invalid
      duty        : effective heater duty in percent (0-100)
      pwm         : raw value on the Arduino pin (0-255);
                    differs from duty*2.55 when INVERTED is true

  Libraries (Library Manager): OneWire, DallasTemperature
  Close the Serial Monitor before running log_serial.py.
*/

#include <OneWire.h>
#include <DallasTemperature.h>

// ---------------- Hardware ----------------
const uint8_t ONE_WIRE_PIN = 2;   // DS18B20 data (4.7k pull-up to 5 V)
const uint8_t HEATER_PIN   = 9;   // PWM pin to the 2N3904 base circuit

// true  : pin HIGH turns the heater OFF (2N3904 inverts the signal)
// false : pin HIGH turns the heater ON
const bool INVERTED = true;

// ---------------- Test profile ----------------
const float         STEP_DUTY_PCT = 100.0;   // heater duty after the baseline
const unsigned long BASELINE_S    = 30;      // heater off for this long

// ---------------- Sampling ----------------
const unsigned long SAMPLE_MS = 1000;        // time between samples
const uint8_t       SENSOR_RESOLUTION = 12;  // 12 bit = 0.0625 degC, ~750 ms conversion

// ---------------- Safety (protective cutoff only) ----------------
const float   MAX_TEMP_C    = 80.0;          // heater latched off above this
const uint8_t MAX_BAD_READS = 3;             // consecutive invalid reads allowed

// ---------------- Internals ----------------
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

unsigned long t0 = 0;              // test start (ms)
unsigned long lastRequest = 0;     // when the last conversion was requested
unsigned long convMs = 0;          // conversion time for the chosen resolution
bool converting = false;

float currentDutyPct = 0.0;
uint8_t currentPwm = 0;            // raw value on the pin
bool faultLatched = false;
uint8_t badReads = 0;

// Apply a heater duty (0-100 %) taking the inversion into account.
void setHeater(float dutyPct) {
  if (dutyPct < 0.0)   dutyPct = 0.0;
  if (dutyPct > 100.0) dutyPct = 100.0;
  uint8_t effective = (uint8_t)(dutyPct * 255.0 / 100.0 + 0.5);
  currentDutyPct = dutyPct;
  currentPwm = INVERTED ? (uint8_t)(255 - effective) : effective;
  analogWrite(HEATER_PIN, currentPwm);
}

void latchFault(const __FlashStringHelper *msg) {
  faultLatched = true;
  setHeater(0.0);
  Serial.println(msg);             // not a CSV row, so the script skips it
}

void logRow(unsigned long ms, bool valid, float tempC) {
  Serial.print(ms);
  Serial.print(F(",nan,"));
  if (valid) Serial.print(tempC, 3);
  else       Serial.print(F("nan"));
  Serial.print(',');
  Serial.print(currentDutyPct, 1);
  Serial.print(',');
  Serial.println(currentPwm);
}

void setup() {
  // Heater OFF before anything else.
  pinMode(HEATER_PIN, OUTPUT);
  setHeater(0.0);

  Serial.begin(9600);              // must match BAUD_RATE in log_serial.py
  Serial.println(F("millis,setpoint,temperature,duty,pwm"));

  sensors.begin();
  if (sensors.getDeviceCount() == 0) {
    latchFault(F("FATAL: no DS18B20 found"));
    return;
  }
  sensors.setResolution(SENSOR_RESOLUTION);
  sensors.setWaitForConversion(false);          // non-blocking reads
  convMs = sensors.millisToWaitForConversion(SENSOR_RESOLUTION) + 20;

  t0 = millis();
  lastRequest = t0 - SAMPLE_MS;                 // first request right away
}

void loop() {
  if (faultLatched) {
    setHeater(0.0);
    return;
  }

  unsigned long now = millis();
  unsigned long elapsedS = (now - t0) / 1000UL;

  // Fixed schedule: off for BASELINE_S, then on forever.
  float wanted = (elapsedS < BASELINE_S) ? 0.0 : STEP_DUTY_PCT;
  if (wanted != currentDutyPct) setHeater(wanted);

  // Start a conversion every SAMPLE_MS.
  if (!converting && (now - lastRequest) >= SAMPLE_MS) {
    lastRequest = now;
    sensors.requestTemperatures();
    converting = true;
  }

  // Read and log once the conversion is ready.
  if (converting && (now - lastRequest) >= convMs) {
    converting = false;
    float t = sensors.getTempCByIndex(0);

    // -127 = disconnected, 85.0 = power-on default (no real conversion)
    bool valid = (t != DEVICE_DISCONNECTED_C) && (t != 85.0);

    if (valid) {
      badReads = 0;
      if (t > MAX_TEMP_C) {
        logRow(now, true, t);
        latchFault(F("FATAL: over-temperature, heater off"));
        return;
      }
    } else {
      badReads++;
      if (badReads >= MAX_BAD_READS) {
        logRow(now, false, 0);
        latchFault(F("FATAL: sensor fault, heater off"));
        return;
      }
    }
    logRow(now, valid, t);
  }
}