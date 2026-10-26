#include "ds18b20_probe.cpp"
#include "heater.cpp"
#include "pi_controller.cpp"
#include "logger.cpp"

// Pins
const uint8_t ONE_WIRE_PIN = 2;   // DS18B20 data line
const uint8_t HEATER_PIN   = 9;   // to the 1k base resistor of the 2N3904

// Timing
const unsigned long TICK_MS = 500;   // 2 Hz control loop
const float TS_S = TICK_MS / 1000.0f;

// PI gains (placeholders: set them from the simulation)
const float KP = 1.0f;
const float KI = 0.0f;

// Maximum duty cycle (raise it after the open-loop test)
const float DUTY_MAX = 0.5f;

// Setpoint and safety: the heater is cut off above setpoint + SAFETY_MARGIN_C
float setpoint = 30.0f;
const float SAFETY_MARGIN_C = 15.0f;

TempSensor    sensor(ONE_WIRE_PIN, 9);
Heater        heater(HEATER_PIN);
PIController  pi(KP, KI, TS_S);
Logger        logger(9600);

unsigned long lastTick = 0;

void setup() {
  heater.begin();
  logger.begin();

  if (!sensor.begin()) {
    Serial.println(F("FATAL: DS18B20 not found. Check wiring."));
    while (true) heater.off();
  }

  if (sensor.conversionTimeMs() >= TICK_MS) {
    Serial.println(F("WARNING: sensor conversion time exceeds the loop period."));
  }

  pi.setOutputLimits(0.0f, DUTY_MAX);
  pi.begin();
  sensor.startConversion();
  lastTick = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTick < TICK_MS) return;
  lastTick += TICK_MS;

  float temperature = sensor.read();
  bool safe = sensor.isValid() && (temperature <= setpoint + SAFETY_MARGIN_C);

  float duty = 0.0f;
  uint8_t pwm;

  if (safe) {
    duty = pi.compute(setpoint, temperature);
    pwm = heater.setDuty(duty);
  } else {
    pwm = heater.off();
    pi.reset();
  }

  logger.logRow(now, setpoint, temperature, duty, pwm);
  sensor.startConversion();
}
