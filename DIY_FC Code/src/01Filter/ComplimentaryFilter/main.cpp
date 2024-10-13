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
#define POL_GYRO_SENS 0.00875f // FS = 250
#define POL_ACC_SENS 0.122/1000f // FS = 4g, 0.122 mg/LSB
#define POL_MAG_SENS_4 1/6842.0f
#define POL_MAG_SENS_12 1/2281.0f

//Unit Conversion
#define PI 3.14159265358979323846f
#define rad2deg 180.0f/PI
#define deg2rad PI/180.0f

// Measurement struct
Measurement_t meas;
quat_t q_est;
vec3_t euler;
float dt = 0.001;

// Function prototypes
void Update_Measurement();
void GyroMagCalibration();


void setup() {
  Serial.begin(115200);
  // Seting pins 24 and 25 to be used as I2C
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
  IMU.enableDefault(); // 1.66 kHz, 2g, 245 dps
  mag.enableDefault();
  mag.writeReg(LIS3MDL::CTRL_REG1, 0x7c); // 1 KHz, high performance mode
  mag.writeReg(LIS3MDL::CTRL_REG2,0x10); // +- 12 gauss
  baro.enableDefault();
  GyroMagCalibration();

}


void loop(){

    // Update the measurement
    Update_Measurement();
    
    // Update the quaternion
    UpdateQ(&meas, dt);
    
    // Get the quaternion
    GetQuaternion(&q_est);
    
    // Get the Euler angles
    GetEulerRPYdeg(&euler, meas.initial_heading);
    
    //Print the Euler angles
    Serial.print("Roll: ");
    Serial.print(euler.x);
    Serial.print(" Pitch: ");
    Serial.print(euler.y);
    Serial.print(" Yaw: ");
    Serial.println(euler.z);


}


void Update_Measurement(){
      // Read IMU data
    IMU.read();
    mag.read();    
    // Convert IMU data
    // Removing the bias from the accelerometer is no bueno. It will cause the filter to diverge
    // meas.acc.x = IMU.a.x * POL_ACC_SENS - meas.acc_bias.x;
    // meas.acc.y = IMU.a.y * POL_ACC_SENS - meas.acc_bias.y;
    // meas.acc.z = IMU.a.z * POL_ACC_SENS - meas.acc_bias.z;
    meas.acc.x = IMU.a.x * POL_ACC_SENS;
    meas.acc.y = IMU.a.y * POL_ACC_SENS;
    meas.acc.z = IMU.a.z * POL_ACC_SENS;
    meas.gyro.x = IMU.g.x * POL_GYRO_SENS * deg2rad-meas.gyro_bias.x;
    meas.gyro.y = IMU.g.y * POL_GYRO_SENS * deg2rad-meas.gyro_bias.y;
    meas.gyro.z = IMU.g.z * POL_GYRO_SENS * deg2rad-meas.gyro_bias.z;
    meas.mag.x = mag.m.x * POL_MAG_SENS_4 - meas.mag_bias.x;
    meas.mag.y = mag.m.y * POL_MAG_SENS_4 - meas.mag_bias.y;
    meas.mag.z = mag.m.z * POL_MAG_SENS_4 - meas.mag_bias.z;
    // meas.mag.x = mag.m.x * POL_MAG_SENS_4;
    // meas.mag.y = mag.m.y * POL_MAG_SENS_4;
    // meas.mag.z = mag.m.z * POL_MAG_SENS_4;
    meas.baro_data.pressure = baro.readPressureMillibars();
    meas.baro_data.temperature = baro.readTemperatureC();
    meas.baro_data.asl = baro.pressureToAltitudeMeters(meas.baro_data.pressure);
}


void GyroMagCalibration(){
    int start_time = millis();
    int num_samples = 0;
    while (millis() - start_time < 10000){
      IMU.read();
      mag.read();
      float x = IMU.g.x * POL_GYRO_SENS * deg2rad;
      float y = IMU.g.y * POL_GYRO_SENS * deg2rad;
      float z = IMU.g.z * POL_GYRO_SENS * deg2rad;
      num_samples++;
      meas.gyro_bias.x += (x- meas.gyro_bias.x)/num_samples;
      meas.gyro_bias.y += (y- meas.gyro_bias.y)/num_samples;
      meas.gyro_bias.z += (z- meas.gyro_bias.z)/num_samples;

      meas.mag_bias.x += (mag.m.x * POL_MAG_SENS_4 - meas.mag_bias.x)/num_samples;
      meas.mag_bias.y += (mag.m.y * POL_MAG_SENS_4 - meas.mag_bias.y)/num_samples;
      meas.mag_bias.z += (mag.m.z * POL_MAG_SENS_4 - meas.mag_bias.z)/num_samples;

      meas.acc_bias.x += (IMU.a.x * POL_ACC_SENS - meas.acc_bias.x)/num_samples;
      meas.acc_bias.y += (IMU.a.y * POL_ACC_SENS - meas.acc_bias.y)/num_samples;
      meas.acc_bias.z += (IMU.a.z * POL_ACC_SENS - meas.acc_bias.z)/num_samples;
    }
    meas.initial_mag.x = meas.mag_bias.x;
    meas.initial_mag.y = meas.mag_bias.y;
    meas.initial_mag.z = meas.mag_bias.z;
    meas.initial_heading = atan2f(meas.initial_mag.y, meas.initial_mag.x);

    // int start_time2 = millis();
    // int num_samples2 = 0;
    // int sum = 0;
    // while (millis() - start_time2 < 10000){
    //   IMU.read();
    //   float x = IMU.g.x * POL_GYRO_SENS * deg2rad;
    //   float y = IMU.g.y * POL_GYRO_SENS * deg2rad;
    //   float z = IMU.g.z * POL_GYRO_SENS * deg2rad;
    //   sum += (x-meas.gyro_bias.x)*(x-meas.gyro_bias.x) + (y-meas.gyro_bias.y)*(y-meas.gyro_bias.y) + (z-meas.gyro_bias.z)*(z-meas.gyro_bias.z);
    //   num_samples2++;
    // }
    // meas.gyro_drift = sqrtf(sum/num_samples2);

    // Serial.print("Gyro Drift: ");
    // Serial.println(meas.gyro_drift);
    Serial.println("Finished Gyro calibration");
    delay(2000);

}