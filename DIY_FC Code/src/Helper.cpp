#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LSM6.h>     // Library for the LSM6DS33
#include <LIS3MDL.h>  // Library for the LIS3MDL
#include <Wire.h>
#include <IntervalTimer.h>  // Allows running a function at a set interval


#define DegToRad(x) ((x)*0.01745329252)  // *pi/180
#define RadToDeg(x) ((x)*57.2957795131)  // *180/pi

#define SEND_DATA 1
#define crsfSerial Serial1 //ELRS RX connectect to Serial1 -- Might change

IntervalTimer IMU_interval;


// Polulu IMU 
/*
#include <LSM6.h>     // Library for the LSM6DS33
#include <LIS3MDL.h>  // Library for the LIS3MDL

LSM6 imuLSM6;        // Object for LSM6 (gyro and accelerometer)
LIS3MDL imuLIS3MDL;  // Object for LIS3MDL (magnetometer)

Gyro sensitivity = 0.00875
Accel sensitivity = 0.000061
Magnetometer sensitivity = 0.146 this will be micro Tesla (uT)
*/

