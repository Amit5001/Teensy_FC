#include <Arduino.h>
#include <Wire.h>
#include <vl53l1x.h>
#include <AlfredoCRSF.h>
#include <SD.h>
#include <HardwareSerial.h>

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
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4
float PWM_val = 0;

float dt = 1.0 / ESC_FREQUENCY ;
// The IMU is connected to I2C 1- we need to initialize Wire.begin() 
// This will be te LiDAR connection.

// The controller will be connected to Serial4. Pin 34 is RX and Pin 35 is TX
#define ELRS_serial Serial8

//**************************************** Functions ****************************************//

void Battery_data(){
    /*AnalogRead returns a value from 0 to 1023, which is then converted to a voltage value.
      Then we need to multiply it by 3.3 to get the actual voltage value.
      Then we use the ration from the voltage divider to get the actual battery voltage.
    */ 

    Voltage = float(analogRead(15)) * float((3.3/1023.0)) * divider_ratio;
    // Serial.println(Voltage);
    Current = float(analogRead(21)) * float((3.3/1023.0)) * current_ratio * 1000.0;
    Serial.println(Current);

    ConsumedCurrent = (Current * 1000.0 / 3600.0)* dt + ConsumedCurrent;
    // Serial.println(ConsumedCurrent);
    RemainedBattery = (StartBattery - ConsumedCurrent)/BatteryMax_mAh * 100.0;
    // Serial.print(RemainedBattery);
}

void saveToSD(){
    DataFile = SD.open("Data.txt", FILE_WRITE);
    if (DataFile){
        DataFile.print(millis());
        DataFile.print(",");
        DataFile.print(float(Voltage));
        DataFile.print(",");
        DataFile.print(float(Current));
        DataFile.print(",");
        DataFile.print(float(RemainedBattery));
        DataFile.print(",");
        DataFile.print(float(ConsumedCurrent));
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
    Serial8.setRX(34);
    Serial8.setTX(35);
    Serial8.begin(CRSF_BAUDRATE, SERIAL_8N1);
    if (!Serial8) {
        Serial.println("Failed to initialize Serial1");
        while (1);
     }
    Serial.println("ELRS initialized");
    ELRS.begin(Serial8);

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD card initialization failed!");
        while(1);
    }
    Serial.println("SD card initialized.");
    
    // Setting the Current and voltage pins:
    pinMode(15, INPUT);
    pinMode(21, INPUT);

    // Setting the motors pins:
    analogWriteFrequency(MOTOR1, ESC_FREQUENCY); // 250 is the frequency for the ESCs
    analogWriteFrequency(MOTOR2, ESC_FREQUENCY); // 250 is the frequency for the ESCs
    analogWriteFrequency(MOTOR3, ESC_FREQUENCY); // 250 is the frequency for the ESCs
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
    if (millis() - log_timer >= log_period){
        log_timer = millis();
        saveToSD();
    }
    Battery_data();
    LiDAR_distance = LiDAR.read();
    // Serial.println(LiDAR_distance);
    ELRS.update();
    throttle = ELRS.getChannel(3);
    // Serial.println(throttle);
    throttle = map(throttle, RC_MIN, RC_MAX, 1100, 1900);
    PWM_val = US_2_PULSE(throttle);
    analogWrite(MOTOR1, PWM_val);
    analogWrite(MOTOR2, PWM_val);
    analogWrite(MOTOR3, PWM_val);
    analogWrite(MOTOR4, PWM_val);
}