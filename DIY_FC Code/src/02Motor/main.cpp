#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AlfredoCRSF.h>

#define MOTOR1_PIN 2
#define FREQUENCY 400

Servo motor1;

void calibrateESC();

void setup() {
  Serial.begin(115200);
  motor1.attach(MOTOR1_PIN, 900, 2000);
  delay(1000);
  Serial.println("Starting Motor1");
  //calibrateESC();
  motor1.writeMicroseconds(1500); // Stop signal. Necessary to arm the ESC
  
}


void loop(){

  motor1.writeMicroseconds(1100);
  Serial.println("Set to 1100");
  delay(2000);
  motor1.writeMicroseconds(1200);
  Serial.println("Set to 1200");
  delay(2000);
  motor1.writeMicroseconds(1300);
  Serial.println("Set to 1300");
  delay(2000);
  motor1.writeMicroseconds(1600);
  Serial.println("Set to 1600");
  delay(2000);
}

void calibrateESC(){
  Serial.println("Calibrating ESC");
  motor1.writeMicroseconds(2000);
  Serial.println("Set to 2000");
  delay(20000);
  motor1.writeMicroseconds(1000);
  Serial.println("Set to 1000");
  delay(10000);
  Serial.println("ESC Calibration Complete");
}