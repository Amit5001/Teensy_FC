#include <Arduino.h>
#include <Wire.h>
#include <vl53l1x.h>
#include <AlfredoCRSF.h>
#include <SD.h>

#define PWM_RESOLUTION 12 // 12-bit resolution
#define ESC_FREQUENCY 250 // 250Hz
#define PWM_PERIOD 1000000/ESC_FREQUENCY
#define PWM_MAX ((1 << PWM_RESOLUTION) - 1) // 4095 for 12-bit
#define PWM_MIN (1000*PWM_MAX) / (1000000/ESC_FREQUENCY)
#define US_2_PULSE(u) ((u)* PWM_MAX / (PWM_PERIOD))
#define RC_MIN 988
#define RC_MAX 2012

// VL53L1X sensor object
VL53L1X LiDAR;

// AlfredoCRSF object
AlfredoCRSF ELRS;

// File for logging:
File DataFile;


// Further variables:
float throttle = 0; // Will be used with ELRS.getChannel(3)
float Voltage, Current, RemainedBattery, StartBattery;
float ConsumedCurrent;
float LiDAR_distance;

// Variables for logging:
unsigned long log_timer = 0;
unsigned long log_period = 100; // 100ms

// Voltage and Current variables:
float R1 = 2200; // Resistors values are from carbon. need to update.
float R2 = 510;
float divider_ratio = R2/(R1+R2);
    /* For the current ratio we calculate the following:
       1. The voltage at the resistor R2 when the current is 1mA is 1/1000*R2 V
       2. The Transistor current sensing ratio 14 A 
       3. The current ratio will be 1./2.
     */
float current_ratio = ((1/1000)*R2)/ 14; 
float PWM_val = 0;


// Motors variables:
#define MOTOR1 2
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 5

//**************************************** Functions ****************************************//

void Battery_data(){
    /*AnalogRead returns a value from 0 to 1023, which is then converted to a voltage value.
      Then we need to multiply it by 3.3 to get the actual voltage value.
      Then we use the ration from the voltage divider to get the actual battery voltage.
    */ 

    Voltage = analogRead(15) * (3.3/1023) * divider_ratio;
    Current = analogRead(21) * (3.3/1023) * current_ratio;
    RemainedBattery = StartBattery - ConsumedCurrent;
}

void saveToSD(){
    DataFile = SD.open("Data.txt", FILE_WRITE);
    if (DataFile){
        DataFile.print(millis());
        DataFile.print(",");
        DataFile.print(Voltage);
        DataFile.print(",");
        DataFile.print(Current);
        DataFile.print(",");
        DataFile.print(RemainedBattery);
        DataFile.print(",");
        DataFile.print(LiDAR_distance);
        DataFile.print(",");
        DataFile.print(throttle);
        DataFile.print(",");
        DataFile.print(PWM_val);
        DataFile.println();
        DataFile.close();
    }
    else {
    Serial.println("Error opening file!");
  }
}

void setup(){
    Serial.begin(115200);
    Wire.begin();
    LiDAR.setTimeout(500);
    if (!LiDAR.init()){
        Serial.println("Failed to detect and initialize LiDAR!");
        while (1);
    }
    LiDAR.setDistanceMode(VL53L1X::Long);
    LiDAR.setMeasurementTimingBudget(50000);
    LiDAR.startContinuous(50);
    ELRS.begin(Serial1);

    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);
    pinMode(MOTOR3, OUTPUT);
    pinMode(MOTOR4, OUTPUT);
    analogWriteFrequency(MOTOR1, 250); // 250 is the frequency for the ESCs
    analogWriteResolution(12);
    
}

void loop(){
    if (millis() - log_timer >= log_period){
        log_timer = millis();
        Battery_data();
        saveToSD();
    }
    LiDAR_distance = LiDAR.read();
    throttle = ELRS.getChannel(3);
    throttle = map(throttle, RC_MIN, RC_MAX, 1100, 1900);
    PWM_val = US_2_PULSE(throttle);
    analogWrite(MOTOR1, PWM_val);
    analogWrite(MOTOR2, PWM_val);
    analogWrite(MOTOR3, PWM_val);
    analogWrite(MOTOR4, PWM_val);
}