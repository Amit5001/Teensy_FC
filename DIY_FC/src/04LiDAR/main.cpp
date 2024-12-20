
#include <Arduino.h>
#include <vl53l1x.h>
#include <Wire.h>



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);  // Wait for Serial Monitor to open

  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400 kHz

}

void loop() {
 

}