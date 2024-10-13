/*
Change this to a struct with the following fields:
- Motor Pin
- Motor PWM

Create a function that will calibrate the motor by:
- Setting the motor to the maximum value
- Waiting for 2 seconds
- Setting the motor to the minimum value
- Waiting for 5 seconds

Create a function that will arm the motor by:
- Setting the motor to the minimum value

Create a function that will disarm the motor by:
- Setting the motors to the off value

Create a function that will set the motor to a specific value by:
- Setting the motor to the value

*/


#include <Arduino.h>
#include <Servo.h>
#include <02Motor/Motor_type.h>
//#include <AlfredoELRS.h>
#include <Wire.h>

// Pin and motor values
#define MOTOR1_PIN 2
#define MOTOR2_PIN 3
#define MOTOR3_PIN 4
#define MOTOR4_PIN 5
#define ESC_FREQUENCY 50
#define ESC_CALIBRATION 1
#define CONTROLLER_ESC_CALIBRATION 0

Motors motors(ESC_FREQUENCY, MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN);
Servo mot1;

void calibrateESC();

void setup() {
  Serial.begin(115200);
  delay(5000);
  motors.Initialization(ESC_CALIBRATION, CONTROLLER_ESC_CALIBRATION);
  delay(5000);
  Serial.println("Starting Motor1");
  // motors.set_motorPWM(motors.motor1, 768);
  // delay(5000);
  // motors.set_motorPWM(motors.motor1, 1060);
  // delay(5000);
  // mot1.attach(MOTOR1_PIN);
  // int pwm = map(1000, motors.MOTOR_MIN, motors.MOTOR_MAX, 0, motors.PWM_MAX);
  // mot1.writeMicroseconds(1000);
  // delay(1000);
  // calibrateESC();

  // mot1.writeMicroseconds(1100);
  // delay(1000);
  // Serial.println("Motor 1 Armed");
}

void loop() {
    // delay(1000);
    // Serial.println("Motor 1 Testing");
    // for (int i = 1100; i < 2000; i+=100){
    //     mot1.writeMicroseconds(i);
    //     Serial.println(i);
    //     delay(1000);
    // }

    // for (int i = 2000; i > 1100; i-=100){
    //     mot1.writeMicroseconds(i);
    //     Serial.println(i);
    //     delay(1000);
    // }
    // mot1.writeMicroseconds(1000);
    // delay(1000);


    motors.set_motorPWM(motors.motor1, 1100);
    delay(5000);
    motors.set_motorPWM(motors.motor1, 1200);
    delay(5000);
    // motors.set_motorPWM(motors.motor1, 1460);
    // delay(5000);



}

void calibrateESC(){
  mot1.writeMicroseconds(2000); // Send max throttle
  delay(2000);                      // Wait for ESC to register max throttle
  mot1.writeMicroseconds(1000); // Send min throttle
  delay(2000);                      // Wait for ESC to register min throttle
  Serial.println("ESC Calibration Complete.");
}