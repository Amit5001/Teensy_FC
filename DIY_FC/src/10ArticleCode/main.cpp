#include <Arduino.h>
#include <Wire.h>
#include <vl53l1x.h>
#include <AlfredoCRSF.h>
#include <SD.h>
#include <HardwareSerial.h>

#define PWM_RESOLUTION 12 // 12-bit resolution
#define ESC_FREQUENCY 500 // 250Hz
#define PWM_PERIOD 1000000/ESC_FREQUENCY
#define PWM_MAX ((1 << PWM_RESOLUTION) - 1) // 4095 for 12-bit
#define PWM_MIN (1000*PWM_MAX) / (1000000/ESC_FREQUENCY)
#define US_2_PULSE(u) ((u)* PWM_MAX / (PWM_PERIOD))
#define RC_MIN 988
#define RC_MAX 2012

// VL53L1X sensor object
VL53L1X LiDAR;
float LiDAR_distance;

// AlfredoCRSF object
AlfredoCRSF ELRS;

// File for logging:
File DataFile;


// Further variables:
int throttle = 0; // Will be used with ELRS.getChannel(3)
float Voltage, Current, RemainedBattery;
float ConsumedCurrent = 0.0;

// Variables for logging:
unsigned long log_timer = 0;
unsigned long log_period = 20; // 100ms

// Voltage and Current variables:
float R1 = 2200.0; // Resistors values are from carbon. need to update.
float R2 = 510.0;
float divider_ratio = (R1+R2)/R2;
    /* For the current ratio we calculate the following:
       1. The voltage at the resistor R2 when the current is 1mA is 1/1000*R2 V
       2. The Transistor current sensing ratio 14 A 
       3. The current ratio will be 1./2.
     */
float current_ratio = ((1.0/1000.0)*R2)/ 14.0; 
float BatteryMax_mAh = 1300.0; // 1300mAh
float StartBattery = BatteryMax_mAh;


// Motors variables:
#define MOTOR1 2
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 5
float PWM_val = 0;

float dt = 1.0 / ESC_FREQUENCY ;
// The IMU is connected to I2C 1- we need to initialize Wire.begin() 
// This will be te LiDAR connection.

//**************************************** Functions ****************************************//

void setup(){
    Serial.begin(420000);
    Wire.begin();


    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1);
    if (!Serial1) {
        Serial.println("Failed to initialize Serial1");
        while (1);
     }
    Serial.println("ELRS initialized");
    ELRS.begin(Serial1);
    delay(100);


    // Setting the motors pins:
    analogWriteFrequency(MOTOR1, ESC_FREQUENCY); // 250 is the frequency for the ESCs
    analogWriteResolution(PWM_RESOLUTION); // 12-bit resolution
    analogWriteFrequency(MOTOR2, ESC_FREQUENCY); // 250 is the frequency for the ESCs
    analogWriteResolution(PWM_RESOLUTION); // 12-bit resolution
    analogWriteFrequency(MOTOR3, ESC_FREQUENCY); // 250 is the frequency for the ESCs
    analogWriteResolution(PWM_RESOLUTION); // 12-bit resolution
    analogWriteFrequency(MOTOR4, ESC_FREQUENCY); // 250 is the frequency for the ESCs
    analogWriteResolution(PWM_RESOLUTION); // 12-bit resolution

    delay(100);
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);
    pinMode(MOTOR3, OUTPUT);
    pinMode(MOTOR4, OUTPUT);
    delay(100);
    
}

void loop(){
    // LiDAR_distance = LiDAR.read();
    // Serial.println(LiDAR_distance);
    ELRS.update();
    throttle = ELRS.getChannel(3);
    Serial.println(throttle);
    throttle = map(throttle, RC_MIN, RC_MAX, 1000, 2000);
    PWM_val = US_2_PULSE(throttle);
    analogWrite(MOTOR1, PWM_val);
    analogWrite(MOTOR2, PWM_val);
    analogWrite(MOTOR3, PWM_val);
    analogWrite(MOTOR4, PWM_val);

}