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


// Motor Pins
#define MOTOR1_PIN 2
#define MOTOR2_PIN 3
#define MOTOR3_PIN 4
#define MOTOR4_PIN 5

// PWM Default Values
#define MOTOR_OFF 0
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define MOTOR_ARM 1100

#define ESC_FREQUENCY 600


void motor1_calibration();


void setup() {
  Serial.begin(115200);
  analogWriteFrequency(MOTOR1_PIN, ESC_FREQUENCY); // 
  analogWriteResolution(12);
  delay(250);

}

void loop() {

    motor1_calibration();



    exit(0);


}


void motor1_calibration(){
  Serial.println("Starting Motor 1 Calibration");
  delay(5000);
  int Value = MOTOR_MAX;
  int PWM = map(Value,1000,2000,0,4095);
  analogWrite(MOTOR1_PIN, PWM);
  analogWrite(MOTOR2_PIN, PWM);
  analogWrite(MOTOR3_PIN, PWM);
  analogWrite(MOTOR4_PIN, PWM);
  delay(2000);

  Value = MOTOR_MIN;
  PWM = map(Value,1000,2000,0,4095);
  analogWrite(MOTOR1_PIN, PWM);
  analogWrite(MOTOR2_PIN, PWM);
  analogWrite(MOTOR3_PIN, PWM);
  analogWrite(MOTOR4_PIN, PWM);
  delay(5000);

}