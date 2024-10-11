#include <Arduino.h>
#include <Var_types.h>
#include <Wire.h>
#include <01Filter/ComplimentaryFilter/CompFilter.h>

// IMU Sensor Libraries
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>

// IMU Sensor Objects
LSM6 IMU;
LIS3MDL mag;
LPS baro;

//IMU Data Conversion
#define POL_GYRO_SENS 0.00875
#define POL_ACC_SENS 1/16384
#define POL_MAG_SENS 1/8000

// Measurement struct
Measurement_t meas;
quat_t q_est;
vec3_t euler;
float dt = 0.01;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize IMU
  if (!IMU.init()){
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
    }
  if (!mag.init()){
    Serial.println("Failed to detect and initialize Magnetometer!");
    while (1);
    }
  if (!baro.init()){
    Serial.println("Failed to detect and initialize Barometer!");
    while (1);
    }
    IMU.enableDefault();
    mag.enableDefault();
    baro.enableDefault();

}


void loop(){

    // Update the measurement
    Update_Measurement();
    
    // Update the quaternion
    UpdateQ(meas, dt);
    
    // Get the quaternion
    GetQuaternion(&q_est);
    
    // Get the Euler angles
    GetEulerRPY(&euler);
    
    
    delay(10);
}


void Update_Measurement(){
      // Read IMU data
    IMU.read();
    mag.read();    
    // Convert IMU data
    meas.acc.x = IMU.a.x * POL_ACC_SENS;
    meas.acc.y = IMU.a.y * POL_ACC_SENS;
    meas.acc.z = IMU.a.z * POL_ACC_SENS;
    meas.gyro.x = IMU.g.x * POL_GYRO_SENS;
    meas.gyro.y = IMU.g.y * POL_GYRO_SENS;
    meas.gyro.z = IMU.g.z * POL_GYRO_SENS;
    meas.mag.x = mag.m.x * POL_MAG_SENS;
    meas.mag.y = mag.m.y * POL_MAG_SENS;
    meas.mag.z = mag.m.z * POL_MAG_SENS;
    meas.baro_data.pressure = baro.readPressureMillibars();
    meas.baro_data.temperature = baro.readTemperatureC();
    meas.baro_data.asl = baro.pressureToAltitudeMeters(meas.baro_data.pressure);
}