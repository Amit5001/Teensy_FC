#include <Arduino.h>
//#include <AlfredoELRS.h>
#include <Wire.h>

// Pin and motor values
#define MOTOR1_PIN 2

#define ESC_FREQUENCY 250

int PWM = 0;


void setup() {
  Serial.begin(115200);
  delay(5000);
  pinMode(MOTOR1_PIN, OUTPUT);
  analogWriteFrequency(MOTOR1_PIN, ESC_FREQUENCY);
  analogWriteResolution(12);

  //Setup motor 1:
//   analogWrite(MOTOR1_PIN, map(1000, 1000, 2000, 1000, 2000));
  PWM = 1000;
  analogWrite(MOTOR1_PIN, map(PWM, 1000, 2000, 1060, 1990));
  Serial.printf("Set to %d\n", PWM);

  delay(1000);
  PWM = 1100;
  analogWrite(MOTOR1_PIN, map(PWM, 1000, 2000, 1060, 1990));
  Serial.printf("Set to %d\n", PWM);
//   analogWrite(MOTOR1_PIN, 1050);
  delay(1000);

}

void loop() {
    PWM = 1300;
    analogWrite(MOTOR1_PIN, PWM);
    Serial.printf("Set to %d\n", PWM);
    delay(2000);
    PWM = 1400;
    analogWrite(MOTOR1_PIN, PWM);
    Serial.printf("Set to %d\n", PWM);
    delay(2000);
    PWM = 1500;
    analogWrite(MOTOR1_PIN, PWM);
    Serial.printf("Set to %d\n", PWM);
    delay(2000);
    PWM = 1550;
    analogWrite(MOTOR1_PIN, PWM);
    Serial.printf("Set to %d\n", PWM);
    delay(2000);
    PWM = 1700;
    analogWrite(MOTOR1_PIN, PWM);
    Serial.printf("Set to %d\n", PWM);
    delay(2000);


}