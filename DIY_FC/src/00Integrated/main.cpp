#include <Arduino.h>

// Filter Library:
#include <01Filter/CompClass.h>

// PID Controller library:
#include <03PID_Loop/PID_type.h>

// Remote Control Library - ELRS:
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

// Motors Library:
#include <02Motor/MotorsControl.h>

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
// Motor 3 FL gpio 4
// Motor 1 FR gpio 2
// Motor 4 BL gpio 5 
// Motor 2 BR gpio 3
#define ELRS_SERIAL 1
#define MOTOR1_PIN 2
#define MOTOR2_PIN 3
#define MOTOR3_PIN 4
#define MOTOR4_PIN 5
#define ESC_FREQUENCY 250

/**** Max Angle and max rate ****/
#define MAX_ANGLE 30.0f
#define MAX_RATE 200.0f
#define CONTROLLER_MIN 988
#define CONTROLLER_MAX 2012


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
PID_out_t PID_stab_out;
PID_out_t PID_rate_out;


/********************************************** Function Prototypes **********************************************/
void Update_Measurement();
void GyroMagCalibration();
void update_controller();
void IMU_init();
void mapping_controller(char);

/********************************************** Main Code **********************************************/

void setup() {
    // Initialize Serial Monitor:
    Serial.begin(115200);
    while (!Serial);
    Serial.println("USB Serial initialized");

    // Initialize ELRS Serial:
    elrsSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
    if (!elrsSerial) {
        Serial.println("Failed to initialize Serial1");
        while (1);
    }
    elrs.begin(elrsSerial);
    Serial.println("ELRS initialized");

    // Initialize IMU:
    IMU_init();

    // Initializing motors and PID:
    initializePIDParams();
    motors.Motors_init();
    GyroMagCalibration();
}

void loop() {
    // Update ELRS data: Reading from the receiver and updating controller_data variable.
    update_controller();

    // Update the measurement:
    Update_Measurement();

    // Update the quaternion:
    Pololu_filter.UpdateQ(&meas, dt);

    // Get the Euler angles:
    Pololu_filter.GetEulerRPYrad(&estimated_attitude, meas.initial_heading);

    // Get the quaternion:
    Pololu_filter.GetQuaternion(&q_est);

    estimated_rate.roll = meas.gyro.x;
    estimated_rate.pitch = meas.gyro.y;
     estimated_rate.yaw = meas.gyro.z;

    if (controller_data.aux1 > 1500){ // Stabilize mode:
        // This mode only need to contain another PID loop for the angle and then the rate.
        mapping_controller('s');
        PID_stab_out = PID_stab(desired_attitude, estimated_attitude, dt);
        PID_rate_out = PID_rate(PID_stab_out.PID_ret, estimated_rate, dt);

    }
    else{ // Acro mode:
        mapping_controller('r');
        PID_rate_out = PID_rate(desired_rate, estimated_rate, dt);
    }
    // Motor Mixing:
    motors.Motor_Mix(PID_rate_out.PID_ret, controller_data.throttle);

    // Set the motor PWM:
    motors.set_motorPWM();



    // Send the data to the Python GUI:
    // UDPSend2Py();
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
      meas.gyro_bias.x += (x - meas.gyro_bias.x)/num_samples;
      meas.gyro_bias.y += (y - meas.gyro_bias.y)/num_samples;
      meas.gyro_bias.z += (z - meas.gyro_bias.z)/num_samples;

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
    controller_data.throttle = elrs.getChannel(1);
    controller_data.roll = elrs.getChannel(2);
    controller_data.pitch = elrs.getChannel(3);
    controller_data.yaw = elrs.getChannel(4);
    controller_data.aux1 = elrs.getChannel(5);
    controller_data.aux1 = elrs.getChannel(6);
    controller_data.aux1 = elrs.getChannel(7);
    controller_data.aux1 = elrs.getChannel(8);

}

void IMU_init(){
        // Seting pins 24 and 25 to be used as I2C
    Wire.begin();
    Wire.setClock(420000);

    // Initialize IMU
    if (!IMU.init()){
        Serial.println("Failed to detect and initialize IMU!");
        while (1);
        }
    if (!mag.init()){
        Serial.println("Failed to detect and initialize Magnetometer!");
        while (1);
        }

    IMU.enableDefault(); // 1.66 kHz, 2g, 245 dps
    IMU.writeReg(LSM6::CTRL2_G, 0b10100000);  // 0b1010 for ODR 1.66 kHz, 0b0000 for 125 dps range
    IMU.writeReg(LSM6::CTRL1_XL, 0b10100000);  // 0b1010 for ODR 1.66 kHz, 0b0000 for 2g range
    // IMU.writeReg(LSM6::CTRL1_XL, 0b10110000); // Set ODR to 3.33 kHz, FS = ±2 g (if unchanged)
    // IMU.writeReg(LSM6::CTRL2_G, 0b10110000);  // Set ODR to 3.33 kHz, FS = 125 dps (if unchanged)

    mag.enableDefault();
    mag.writeReg(LIS3MDL::CTRL_REG1, 0b11100110); // 1 KHz, high performance mode
    mag.writeReg(LIS3MDL::CTRL_REG2,0x10); // +- 4 gauss
}

void mapping_controller(char state){
    if (state == 's'){ // Mapping the controller input into desired angle:
        desired_attitude.roll = map(controller_data.roll, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_ANGLE, MAX_ANGLE);
        desired_attitude.pitch = map(controller_data.pitch, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_ANGLE, MAX_ANGLE);
        desired_attitude.yaw = map(controller_data.yaw, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_ANGLE, MAX_ANGLE);
    }
    else if (state == 'r'){ // Mapping the controller input into desired rate:
        desired_rate.roll = map(controller_data.roll, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_RATE, MAX_RATE);
        desired_rate.pitch = map(controller_data.pitch, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_RATE, MAX_RATE);
        desired_rate.yaw = map(controller_data.yaw, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_RATE, MAX_RATE);
    }
}