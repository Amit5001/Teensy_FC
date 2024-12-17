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



#include <Arduino.h>

const int motorPin = 12;  // GPIO12 (D12) for the motor signal

int motor1_value = 1100;

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(115200);
  calibrate_motor(1000);  // Adjust pulse width for speed (500-2500 microseconds)
  Serial.println("1000");



}

void loop() {

  writeMotorSpeed(motor1_value,motorPin);

}




// Function to start the motor 
void calibrate_motor(int pulseWidth) {
  // Send PWM pulse for continuous operation with a 20ms period
  for (int i = 0; i < 50; i++) {  // Repeat to maintain signal
    digitalWrite(motorPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(motorPin, LOW);
    delayMicroseconds(20000 - pulseWidth);
  }
}

// Function that run on the main loop 
void writeMotorSpeed(int pulseWidth,int motorPin) {
    digitalWrite(motorPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(motorPin, LOW);
    delayMicroseconds(20000 - pulseWidth);
  
  }
