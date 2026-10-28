// DS18B20 diagnostic sketch. Upload alone (not thermobox.ino), open Serial Monitor at 9600.
// Prints each reading and a running summary of how often the sensor fails.

#include <OneWire.h>
#include <DallasTemperature.h>

const uint8_t SENSOR_PIN = 2;      // <-- set to your DS18B20 data pin
const int8_t  HEATER_PIN = -1;     // <-- set to the heater pin to force it OFF, or -1 if unused
const uint8_t RESOLUTION = 12;     // 9..12 bits
const unsigned long PERIOD_MS = 1000;
const uint16_t SUMMARY_EVERY = 20; // print summary every N readings

OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress addr;

unsigned long total = 0, good = 0, disconnected = 0, powerOn85 = 0, outOfRange = 0;
unsigned long sameCount = 0;
float lastT = NAN, minT = 1000, maxT = -1000;

void printAddress(const DeviceAddress a) {
  for (uint8_t i = 0; i < 8; i++) {
    if (a[i] < 16) Serial.print('0');
    Serial.print(a[i], HEX);
  }
}

void summary() {
  Serial.println(F("---- summary ----"));
  Serial.print(F("readings: ")); Serial.print(total);
  Serial.print(F("  good: ")); Serial.print(good);
  Serial.print(F(" ("));
  Serial.print(total ? 100.0f * good / total : 0.0f, 1);
  Serial.println(F("%)"));
  Serial.print(F("disconnected/CRC (-127): ")); Serial.println(disconnected);
  Serial.print(F("power-on value (85.0):   ")); Serial.println(powerOn85);
  Serial.print(F("out of range:            ")); Serial.println(outOfRange);
  if (good) {
    Serial.print(F("min/max: ")); Serial.print(minT, 2);
    Serial.print(F(" / ")); Serial.println(maxT, 2);
  }
  Serial.println(F("-----------------"));
}

void setup() {
  if (HEATER_PIN >= 0) {            // inverting stage: HIGH = heater off
    pinMode(HEATER_PIN, OUTPUT);
    digitalWrite(HEATER_PIN, HIGH);
  }

  Serial.begin(9600);
  Serial.println(F("DS18B20 test"));

  sensors.begin();
  Serial.print(F("Devices found: ")); Serial.println(sensors.getDeviceCount());
  Serial.print(F("Parasite power: ")); Serial.println(sensors.isParasitePowerMode() ? F("YES") : F("no"));

  if (!sensors.getAddress(addr, 0)) {
    Serial.println(F("FAIL: no sensor found. Check wiring, 4.7k pull-up (data to 5V), and SENSOR_PIN."));
    while (true) delay(1000);
  }
  Serial.print(F("Address: ")); printAddress(addr); Serial.println();
  Serial.print(F("Family code 0x28 (DS18B20): "));
  Serial.println(addr[0] == 0x28 ? F("yes") : F("NO - different chip"));

  sensors.setResolution(addr, RESOLUTION);
  Serial.print(F("Resolution: ")); Serial.println(sensors.getResolution(addr));
  Serial.println(F("\nmillis,temperature,status"));
}

void loop() {
  unsigned long start = millis();

  sensors.requestTemperatures();            // blocks for the conversion time
  float t = sensors.getTempC(addr);
  total++;

  const char* status = "OK";
  if (t == DEVICE_DISCONNECTED_C) { status = "DISCONNECTED_OR_CRC"; disconnected++; }
  else if (t == 85.0f)            { status = "POWER_ON_85";         powerOn85++; }
  else if (t < -10.0f || t > 125.0f) { status = "OUT_OF_RANGE";     outOfRange++; }
  else {
    good++;
    if (t < minT) minT = t;
    if (t > maxT) maxT = t;
    // A real sensor jitters by at least one LSB now and then; long identical runs are suspicious.
    sameCount = (t == lastT) ? sameCount + 1 : 0;
    lastT = t;
    if (sameCount >= 30) status = "OK_BUT_FROZEN?";
  }

  Serial.print(millis()); Serial.print(',');
  Serial.print(t, 2);     Serial.print(',');
  Serial.println(status);

  if (total % SUMMARY_EVERY == 0) summary();

  while (millis() - start < PERIOD_MS) { }
}
