#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature library
DallasTemperature sensors(&oneWire);

void setup(void) {
  // Start serial communication for debugging
  Serial.begin(9600);
  Serial.println("DS18B20 Temperature Sensor Test");

  // Start up the library
  sensors.begin();
}

void loop(void) { 
  // Send the command to all sensors on the line to get temperatures
  sensors.requestTemperatures(); 
  
  // Fetch the temperature in Celsius
  float tempC = sensors.getTempCByIndex(0);

  // Check if the reading is valid (-127.00 means a connection issue)
  if(tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data. Check your wiring!");
  } else {
    // Print Celsius
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print(" °C  |  ");
    
    // Convert and print Fahrenheit
    float tempF = DallasTemperature::toFahrenheit(tempC);
    Serial.print(tempF);
    Serial.println(" °F");
  }
  
  // Wait 2 seconds before the next reading
  delay(2000);
}
