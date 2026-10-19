#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2 // Change to your data pin

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long startTime;
unsigned long endTime;
int sampleCount = 0;
const int TEST_SAMPLES = 50; // Total samples to measure for averaging

void setup() {
  Serial.begin(115200);
  sensors.begin();
  
  // Set resolution: 9 (fastest), 10, 11, or 12 (most precise)
  sensors.setResolution(12); 
  
  // Disable blocking delay so we can measure exact timing manually
  sensors.setWaitForConversion(false); 
  
  Serial.println("--- Starting DS18B20 Sample Rate Test ---");
  Serial.print("Resolution set to: ");
  Serial.print(sensors.getResolution());
  Serial.println(" bits");
  
  startTime = millis();
  
  // Request the very first conversion to prime the loop
  sensors.requestTemperatures(); 
}

void loop() {
  if (sampleCount < TEST_SAMPLES) {
    // Check if the sensor has finished converting the temperature
    if (sensors.isConversionComplete()) {
      float tempC = sensors.getTempCByIndex(0); // Read the temperature
      sampleCount++;
      
      // Immediately start the next conversion
      if (sampleCount < TEST_SAMPLES) {
        sensors.requestTemperatures();
      }
    }
  } else {
    // Test complete, calculate results
    endTime = millis();
    unsigned long totalTime = endTime - startTime;
    float averageTimePerSample = (float)totalTime / TEST_SAMPLES;
    float sampleRateHz = 1000.0 / averageTimePerSample;
    
    Serial.println("\n--- Test Results ---");
    Serial.print("Total Time for "); Serial.print(TEST_SAMPLES); Serial.print(" samples: "); Serial.print(totalTime); Serial.println(" ms");
    Serial.print("Average Time per Sample: "); Serial.print(averageTimePerSample); Serial.println(" ms");
    Serial.print("Actual Sample Rate: "); Serial.print(sampleRateHz); Serial.println(" Hz");
    
    while(true); // Stop execution
  }
}
