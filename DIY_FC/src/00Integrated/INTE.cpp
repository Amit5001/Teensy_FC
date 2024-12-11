#include <Arduino.h>

// Filter Library:
#include <CompClass.h>

// PID Controller library:
#include <PID_type.h>

// Remote Control Library - ELRS:
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

// Motors Library:
#include <MotorsControl.h>

// Exotic Variables defined here:
#include <Var_types.h>

// IMU Sensor Libraries:
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>


// UDP Communication Libraries:
#include <00UDP/Rtes.h>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <vector>
#include <iomanip>
#include <cmath>

/********************************************** Definitions **********************************************/
#define ELRS_SERIAL 1
#define MOTOR1_PIN 2
#define MOTOR2_PIN 3
#define MOTOR3_PIN 4
#define MOTOR4_PIN 5
#define ESC_FREQUENCY 250

/**** IMU Data parameters:  ****/
#define POL_GYRO_SENS 4.375/1000.0f    // FS = 125
#define POL_ACC_SENS 0.061/1000.0f     // FS = 2g, 0.061 mg/LSB
#define POL_MAG_SENS 1/6842.0f         // FS = 4 gauss
// #define POL_GYRO_SENS 0.00875f      // FS = 250
// #define POL_ACC_SENS 0.122/1000.0f  // FS = 4g, 0.122 mg/LSB
// #define POL_MAG_SENS 1/2281.0f      // FS = 8 gauss



/**** Unit Conversion parameters: ****/
#define PI 3.14159265358979323846f
#define rad2deg 180.0f/PI
#define deg2rad PI/180.0f


/**** UDP Communication: ****/
#define UDP_BRIDGE 1 // 1 to make the bridge of the data 
#define SEND_DATA 1 // send data 
#define PP Serial.println("")
#define MPU_TICK_RATE 10000
#define MAG 'm'
#define POLOLU 'p'
#define euler 'e'
#define RC 'r'
#define RC_ros 'n'
#define Quaternion 'q'

#define elrsSerial Serial1  // Use Serial1 for the CRSF communication


/********************************************** Variables Decleration **********************************************/

// ELRS Controller:
AlfredoCRSF elrs;
Controller_s controller_data;
uint16_t controller[16];  // RC channel values



// Motors Variables:
Motors motors(ESC_FREQUENCY, MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN);

// IMU and Filter Variables:
LSM6 IMU;
LIS3MDL mag;
LPS baro;


// Measurement struct
Measurement_t meas;
quat_t q_est;
vec3_t euler_angles;

// Filter Object:
CompFilter Pololu_filter(true); // True for enabling the magnetometer
float dt = 1/1100.0f;

// Desired Attitude - From the controller:
attitude_t desired_attitude;
long mid_value = 1500;
attitude_t motor_input;
attitude_t desired_rate;
attitude_t estimated_attitude;
attitude_t estimated_rate;

/********************************************** Function Prototypes **********************************************/
void Update_Measurement();
void GyroMagCalibration();
void update_controller();

/********************************************** Main Code **********************************************/

void setup() {
    // Initialize Serial Monitor:
    Serial.begin(115200);
    while (!Serial);
    Serial.println("USB Serial initialized");

    // Initialize ELRS Serial:
    elrsSerial.begin(420000);
    if (!elrsSerial) {
        Serial.println("Failed to initialize Serial1");
        while (1);
    }

    // Initialize ELRS:
    elrs.begin(elrsSerial);
    Serial.println("ELRS initialized");

    // Initializing motors and PID:
    initializePIDParams();
    motors.Motors_init();
}

void loop() {
    // Update ELRS data: Reading from the receiver and updating controller_data variable.
    update_controller();

    // Update the measurement:
    Update_Measurement();

    // Update the quaternion:
    Pololu_filter.UpdateQ(&meas, dt);

    // Get the Euler angles:
    Pololu_filter.GetEulerRPYrad(&euler_angles, meas.initial_heading);

    // Get the quaternion:
    Pololu_filter.GetQuaternion(&q_est);

    // PID Controller:  Might want to add a if statement to change between rate and stabilize mode.
    estimated_attitude.roll = euler_angles.x;
    estimated_attitude.pitch = euler_angles.y;
    estimated_attitude.yaw = euler_angles.z;
    desired_rate = PID_stab(desired_attitude, estimated_attitude, meas.gyro, dt);

    // PID Controller for Rate:
    estimated_rate.roll = meas.gyro.x;
    estimated_rate.pitch = meas.gyro.y;
    estimated_rate.yaw = meas.gyro.z;
    motor_input = PID_rate(desired_rate, estimated_rate, dt); // Need to change the gyro measurements to the estimated attitude rates.

    // Motor Mixing:
    motors.Motor_Mix(motor_input, controller_data.throttle);

    // Set the motor PWM:
    motors.set_motorPWM();



    // Send the data to the Python GUI:
    UDPSend2Py();
}


/********************************************** Function Definitions **********************************************/


void Update_Measurement(){
    // Read IMU data
    IMU.read();
    mag.read();    
    meas.acc.x = IMU.a.x * POL_ACC_SENS;
    meas.acc.y = IMU.a.y * POL_ACC_SENS;
    meas.acc.z = IMU.a.z * POL_ACC_SENS;
    meas.gyro.x = IMU.g.x * POL_GYRO_SENS * deg2rad-meas.gyro_bias.x;
    meas.gyro.y = IMU.g.y * POL_GYRO_SENS * deg2rad-meas.gyro_bias.y;
    meas.gyro.z = IMU.g.z * POL_GYRO_SENS * deg2rad-meas.gyro_bias.z;
    meas.mag.x = mag.m.x * POL_MAG_SENS - meas.mag_bias.x;
    meas.mag.y = mag.m.y * POL_MAG_SENS - meas.mag_bias.y;
    meas.mag.z = mag.m.z * POL_MAG_SENS - meas.mag_bias.z;
    // meas.baro_data.pressure = baro.readPressureMillibars();
    // meas.baro_data.temperature = baro.readTemperatureC();
    // meas.baro_data.asl = baro.pressureToAltitudeMeters(meas.baro_data.pressure);
}


void GyroMagCalibration(){
    Serial.println("Starting Gyro calibration");
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

      meas.mag_bias.x += (mag.m.x * POL_MAG_SENS - meas.mag_bias.x)/num_samples;
      meas.mag_bias.y += (mag.m.y * POL_MAG_SENS - meas.mag_bias.y)/num_samples;
      meas.mag_bias.z += (mag.m.z * POL_MAG_SENS - meas.mag_bias.z)/num_samples;

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

void update_controller(){
    // Update the controller data:
    elrs.update();
    // Get the controller data:
    controller_data.throttle = elrs.getchannel(1);
    controller_data.roll = elrs.getchannel(2);
    controller_data.pitch = elrs.getchannel(3);
    controller_data.yaw = elrs.getchannel(4);
    controller_data.aux1 = elrs.getchannel(5);
    controller_data.aux1 = elrs.getchannel(6);
    controller_data.aux1 = elrs.getchannel(7);
    controller_data.aux1 = elrs.getchannel(8);

}