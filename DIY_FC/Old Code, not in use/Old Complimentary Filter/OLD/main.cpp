#include <Arduino.h>
// #include <AlfredoCRSF.h>
// #include <HardwareSerial.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <LSM6.h>     // Library for the LSM6DS33
#include <LIS3MDL.h>  // Library for the LIS3MDL
#include <Wire.h>
#include <vector_type.h>
#include <IMUCombiner.h>

#define DegToRad(x) ((x)*0.01745329252)  // *pi/180
#define RadToDeg(x) ((x)*57.2957795131)  // *180/pi

#define SEND_DATA 1
#define crsfSerial Serial1 //ELRS RX connectect to Serial1 -- Might change

// Define IMU instance:
// Adafruit_MPU6050 MPU6050;
LSM6 IMU_Pol;        // Object for LSM6 (gyro and accelerometer)
LIS3MDL Pol_Mag;  // Object for LIS3MDL (magnetometer)

#define POL_GYRO_SENS 0.00875
#define POL_ACC_SENS 1/16384
#define POL_MAG_SENS 1/8000

// IMU data
vec3_t acc;
vec3_t gyro;
vec3_t mag;
float est[4];

IMUCombiner<4, 4> KF(1,1,1); // () Gains for P, Q, R <> Nmeas and Nstate

//--------------- CRSF RX ----------------
// AlfredoCRSF CRSF_RX;


void getIMUData(){
  // Get the data from the IMU
  // Get the data from the Pololu IMU
  IMU_Pol.read();
  Pol_Mag.read();

  acc.x = IMU_Pol.a.x * POL_ACC_SENS;
  acc.y = IMU_Pol.a.y * POL_ACC_SENS;
  acc.z = IMU_Pol.a.z * POL_ACC_SENS;
  gyro.x = IMU_Pol.g.x * POL_GYRO_SENS;
  gyro.y = IMU_Pol.g.y * POL_GYRO_SENS;
  gyro.z = IMU_Pol.g.z * POL_GYRO_SENS;
  mag.x = Pol_Mag.m.x * POL_MAG_SENS;
  mag.y = Pol_Mag.m.y * POL_MAG_SENS;

  // Set data to parameters:
  // float POL_data[6] = {static_cast<float>(IMU_Pol.a.x), static_cast<float>(IMU_Pol.a.y), static_cast<float>(IMU_Pol.a.z), static_cast<float>(IMU_Pol.g.x), static_cast<float>(IMU_Pol.g.y), static_cast<float>(IMU_Pol.g.z)};
  // float MPU_data[6] = {a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z};
  // float MAG_data[3] = {static_cast<float>(Pol_Mag.m.x), static_cast<float>(Pol_Mag.m.y), static_cast<float>(Pol_Mag.m.z)};

  // Send data to Kalman Filter
  float* result = KF.one_imu(acc, gyro);
  memcpy(est, result, sizeof(est));


}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize the IMU
  if (!IMU_Pol.init()){
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  if (!Pol_Mag.init()){
    Serial.println("Failed to detect and initialize Magnetometer!");
    while (1);
  }
  IMU_Pol.enableDefault();
  Pol_Mag.enableDefault();

}


void loop() {

  getIMUData();
  float roll = atan2(acc.y,acc.z) ; // Roll in radians
  float pitch = atan2(acc.x,sqrt(pow(acc.y,2) + pow(acc.z,2))); // Pitch in radians
  Serial.print(" Unfiltered Attitude: "); Serial.print(roll); Serial.print(" "); Serial.println(pitch);
  Serial.print(" Filtered Attitude:   "); Serial.print(est[0]); Serial.print(" "); Serial.println(est[1]);
  Serial.println(" ");
  Serial.print(" Unfiltered Gyro: "); Serial.print(gyro.x); Serial.print(" "); Serial.println(gyro.y);
  Serial.print(" Filtered Gyro:   "); Serial.print(est[2]); Serial.print(" "); Serial.println(est[3]);
  Serial.println(" ");

  
  delay(500);
  
}