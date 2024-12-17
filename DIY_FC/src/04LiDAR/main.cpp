
#include <Arduino.h>
#include <vl53l8cx.h>
#include <Wire.h>

#define LPN_PIN 23

VL53L8CX lidar(&Wire, LPN_PIN);  // LiDAR object
uint8_t res = VL53L8CX_RESOLUTION_4X4;  // LiDAR resolution
uint8_t ranging_mode = VL53L8CX_RANGING_MODE_CONTINUOUS;  // LiDAR ranging mode


void tof_init(){
 
  // Init and start sensor
  lidar.begin();

  // Setting LPN pin to HIGH
  pinMode(LPN_PIN, OUTPUT);
  digitalWrite(LPN_PIN, HIGH);
  
  Serial.println("Waiting for sensor to be detected...");
  while (!lidar.init()) {
    Serial.println("Sensor not detected, check wiring");
    delay(100);
  }
  Serial.println("Sensor detected!");

  // Resolution
  lidar.set_resolution(res);

  // Ranging mode
  lidar.set_ranging_mode(ranging_mode);

  // Start measurements
  lidar.start_ranging();
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);  // Wait for Serial Monitor to open

  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400 kHz
  
  tof_init();
  Serial.println("Setup done!");
}

void loop() {
  // Printing I2C addresses
  Serial.println("I2C addresses:");
  for (uint8_t i = 0; i < 128; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("0x");
      Serial.println(i, HEX);
    }
  }
  delay(1000);
  Serial.println("Finished scanning I2C addresses");

  // VL53L8CX_ResultsData results;
  // uint8_t data_ready = 0;
  // uint8_t status;

}