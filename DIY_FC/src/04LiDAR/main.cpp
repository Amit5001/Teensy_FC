
#include <Arduino.h>
#include <vl53l1x.h>
#include <Wire.h>
#include <SPI.h>
#include <Bitcraze_PMW3901.h>

// VL53L1X sensor object
VL53L1X sensor;

// PMW3901 sensor object
#define CSpin 10
Bitcraze_PMW3901 flow(CSpin);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);  // Wait for Serial Monitor to open

  // Initialize I2C communication
  Wire1.begin();
  Wire1.setClock(400000); // Set I2C clock to 400 kHz

  // Initializing FlowDeck sensor:
  sensor.setBus(&Wire1);
  if(!flow.begin()){
    Serial.println("Failed to initialize FlowDeck sensor!");
    while(1);
  }
  Serial.println("FlowDeck sensor initialized!");

  // Initialize VL53L1X sensor
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // Set the sensor to long range mode
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
  Serial.println("VL53L1X sensor initialized!");
}

void loop() {
  int16_t deltaX, deltaY;

  // Read the flow sensor
  flow.readMotionCount(&deltaX, &deltaY);

  // Print the motion count
  Serial.print("Delta X: ");
  Serial.print(deltaX);
  Serial.print(", Delta Y: ");
  Serial.println(deltaY);

  // Read the VL53L1X sensor
  uint16_t distance = sensor.read();

  // Print the distance
  Serial.print("Distance: ");
  Serial.println(distance);

  // Delay for 100 ms
  delay(100);
}