// Filter Library:
#include <01Filter/CompClass.h>
#include <01Filter/EkfClass.h>
#include <01Filter/Madgwick.h>
#include <01Filter/STD_Filter.h>

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
#include <00UDP/Drone_com.h>


/********************************************** Definitions **********************************************/

#define ELRS_SERIAL 1
#define MOTOR1_PIN 2
#define MOTOR2_PIN 3
#define MOTOR3_PIN 4
#define MOTOR4_PIN 5

/**** Max Angle and max rate ****/
#define MAX_ANGLE 15.0f
#define MAX_RATE 200.0f
#define CONTROLLER_MIN 988
#define CONTROLLER_MAX 2012
#define CONTROLLER_MID 1500

// DeadBand for the controller throttle:
#define CONTROLL_THR_MAX 1520
#define CONTROLL_THR_MIN 1480

/**** IMU Data parameters:  ****/
#define POL_GYRO_SENS 17.5/1000.0f    // FS = 500
#define POL_ACC_SENS 0.061/1000.0f     // FS = 2g, 0.061 mg/LSB
#define POL_MAG_SENS 1/6842.0f         // FS = 4 gauss
#define IMU_THRESHOLD 0.05f            // Threshold for the IMU data to be considered valid


#define elrsSerial Serial1  // Use Serial1 for the CRSF communication


/********************************************** Variables Decleration **********************************************/

// ELRS Controller:
AlfredoCRSF elrs;
Controller_s controller_data;


// Motors Variables:
Motors motors(MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN);
motor_t motor_pwm;
bool is_armed = false;
bool THR_FS = false;

// IMU and Filter Variables:
LSM6 IMU;
LIS3MDL mag;
LPS baro;


// Measurement struct
Measurement_t meas;
quat_t q_est;

// Desired Attitude - From the controller:
attitude_t desired_attitude;
attitude_t motor_input; // Currently not in use. replaced by PID_rate_out.PID_ret
attitude_t desired_rate;
attitude_t estimated_attitude;
attitude_t estimated_rate;
PID_out_t PID_stab_out;
PID_out_t PID_rate_out;
float t_PID_s = 0.0f;
float t_PID_r = 0.0f;
double actual_dt = 0.0f;

// Filters:
CompFilter Comp_filter(false); // True for enabling the magnetometer
EKF Ekf_filter(&meas, DT);
Madgwick madgwick_filter(&meas, &estimated_attitude, &q_est, SAMPLE_RATE, 0.9f); // Beta value is set to 0.1
STD_Filter std_filter(&meas, DT); // Standard filter for the IMU data
int filter_type = 0; // 0 for Complementary, 1 for Kalman

// Timers periods:
const unsigned long PWM_PERIOD_1 = 1000000 / ESC_FREQUENCY; // 1,000,000 us / frequency in Hz. Recieving PWM signal every 2ms -- NOT REALLY NECESSARY, we have the same variable at motors.h
const unsigned long STAB_PERIOD = 1000000 / (ESC_FREQUENCY/2); // 300 Hz period in microseconds
const unsigned long IMU_PERIOD = 1000000 / SAMPLE_RATE;
const unsigned long DATA_PERIOD = 1000000 / 50; // 50 Hz period in microseconds

// Timers:
elapsedMicros motor_timer;
elapsedMicros stab_timer;
elapsedMicros imu_timer;
elapsedMicros Data_timer;


/********************************************** Function Prototypes **********************************************/
void Update_Measurement();
void GyroMagCalibration();
void update_controller();
void IMU_init();
void mapping_controller(char);
void resetMicrocontroller();
void check_arming_state();
void controller_trheshold();
void compclass_function();
void channel_estimated();
void filter_method();

/*********************************************** Main Code ***********************************************/

void setup() {
    // Initialize Serial Monitor:
    Serial.begin(115200);
    while (!Serial);
    Serial.println("USB Serial initialized");

    DRON_COM::init_com();

    Serial.println("UDP initialized");

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
    GyroMagCalibration(); 
    motors.Motors_init();

}

void loop() {
    // Update ELRS data: Reading from the receiver and updating controller_data variable.
    update_controller();

    // Check arming:
    check_arming_state();

    if (imu_timer >= IMU_PERIOD){
        actual_dt = (double)imu_timer / 1000000.0f;
        // Update the measurement:
        Update_Measurement();

        // Getting filtered data:
        Comp_filter.UpdateQ(&meas, actual_dt/2);
        Comp_filter.GetEulerRPYdeg(&estimated_attitude, meas.initial_heading);
        Comp_filter.GetQuaternion(&q_est);

        Serial.println(estimated_attitude.roll);

        if (is_armed){
            // Get Actual rates:
            estimated_rate.roll = meas.gyro_LPF.x;
            estimated_rate.pitch = meas.gyro_LPF.y;
            estimated_rate.yaw = meas.gyro_LPF.z;

            if ((controller_data.aux1 > 1500) && (stab_timer >= STAB_PERIOD)){ // Stabilize mode:
                    // Calculating dt for the PID- in seconds:
                    t_PID_s = (float)stab_timer / 1000000.0f;
                    mapping_controller('s');
                    PID_stab_out = PID_stab(desired_attitude, estimated_attitude, t_PID_s);

                    PID_stab_out.PID_ret.pitch = -1 * PID_stab_out.PID_ret.pitch;
                    desired_rate = PID_stab_out.PID_ret;
                    desired_rate.yaw = map(controller_data.yaw, CONTROLLER_MIN, CONTROLLER_MAX, MAX_RATE, -MAX_RATE);
                    stab_timer = 0;
            }
            else if (controller_data.aux1 < 1500) { // Acro mode:
                mapping_controller('r');
            }

            // Set the motor PWM:
            if ((controller_data.throttle > 1000) && (motor_timer >= PWM_PERIOD)){
                t_PID_r = (float)motor_timer / 1000000.0f;
                PID_rate_out = PID_rate(desired_rate, estimated_rate, t_PID_r);
                motors.Motor_Mix(PID_rate_out.PID_ret, controller_data.throttle);
                motors.set_motorPWM();
                motor_timer = 0;

            }
        
            //Getting the motors struct to send data back:
            motor_pwm = motors.Get_motor();
        }

        // Sending new UDP Packet:
        if (Data_timer >= DATA_PERIOD){
            DRON_COM::convert_Measurment_to_byte(meas,
                                                q_est, desired_attitude,
                                                motor_pwm, desired_rate,
                                                estimated_attitude, estimated_rate,
                                                PID_stab_out, PID_rate_out, controller_data);

            DRON_COM::send_data();
            Data_timer = 0;
        }
        
        imu_timer = 0;
    }
}


/********************************************** Function Definitions **********************************************/


void Update_Measurement(){
    // Read IMU data
    IMU.read();
    mag.read();    
    meas.acc.x = IMU.a.x * POL_ACC_SENS - meas.acc_bias.x;
    meas.acc.y = IMU.a.y * POL_ACC_SENS - meas.acc_bias.y;
    meas.acc.z = IMU.a.z * POL_ACC_SENS - meas.acc_bias.z;
    // meas.acc.x = IMU.a.x * POL_ACC_SENS;
    // meas.acc.y = IMU.a.y * POL_ACC_SENS;
    // meas.acc.z = IMU.a.z * POL_ACC_SENS;
    if (abs(meas.acc.x) < IMU_THRESHOLD) { meas.acc.x = 0;}
    if (abs(meas.acc.y) < IMU_THRESHOLD) { meas.acc.y = 0;}
    if (abs(meas.acc.z) < IMU_THRESHOLD) { meas.acc.z = 0;}

    meas.gyroDEG.x = IMU.g.x * POL_GYRO_SENS - meas.gyro_bias.x;
    meas.gyroDEG.y = IMU.g.y * POL_GYRO_SENS - meas.gyro_bias.y;
    meas.gyroDEG.z = IMU.g.z * POL_GYRO_SENS - meas.gyro_bias.z;
    meas.gyroRAD.x = meas.gyroDEG.x * deg2rad;
    meas.gyroRAD.y = meas.gyroDEG.y * deg2rad;
    meas.gyroRAD.z = meas.gyroDEG.z * deg2rad;

    meas.mag.x = mag.m.x * POL_MAG_SENS - meas.mag_bias.x;
    meas.mag.y = mag.m.y * POL_MAG_SENS - meas.mag_bias.y;
    meas.mag.z = mag.m.z * POL_MAG_SENS - meas.mag_bias.z;
    if (abs(meas.mag.x < IMU_THRESHOLD)) { meas.mag.x = 0;}
    if (abs(meas.mag.y < IMU_THRESHOLD)) { meas.mag.y = 0;}
    if (abs(meas.mag.z < IMU_THRESHOLD)) { meas.mag.z = 0;}

}


void GyroMagCalibration(){
    Serial.println("Starting Gyro calibration");
    int start_time = millis();
    int num_samples = 0;
    while (millis() - start_time < 10000){
      IMU.read();
      mag.read();
      float x = IMU.g.x * POL_GYRO_SENS;
      float y = IMU.g.y * POL_GYRO_SENS;
      float z = IMU.g.z * POL_GYRO_SENS;
      num_samples++;
      meas.gyro_bias.x += (x - meas.gyro_bias.x)/num_samples;
      meas.gyro_bias.y += (y - meas.gyro_bias.y)/num_samples;
      meas.gyro_bias.z += (z - meas.gyro_bias.z)/num_samples;

      meas.mag_bias.x += (mag.m.x * POL_MAG_SENS - meas.mag_bias.x)/num_samples;
      meas.mag_bias.y += (mag.m.y * POL_MAG_SENS - meas.mag_bias.y)/num_samples;
      meas.mag_bias.z += (mag.m.z * POL_MAG_SENS - meas.mag_bias.z)/num_samples;

      meas.acc_bias.x += (IMU.a.x * POL_ACC_SENS - meas.acc_bias.x)/num_samples;
      meas.acc_bias.y += (IMU.a.y * POL_ACC_SENS - meas.acc_bias.y)/num_samples;
    //   meas.acc_bias.z += (IMU.a.z * POL_ACC_SENS - meas.acc_bias.z)/num_samples;
    }
    meas.initial_mag.x = meas.mag_bias.x;
    meas.initial_mag.y = meas.mag_bias.y;
    meas.initial_mag.z = meas.mag_bias.z;
    meas.initial_heading = atan2f(meas.initial_mag.y, meas.initial_mag.x);

    // meas.acc_bias.z += 1.0f; //  Adding back 1g, so the bias will remove only noise around 1g.
    
    Serial.println("Finished Gyro calibration");
    delay(2000);

}

void update_controller(){
    // Update the controller data:
    elrs.update();
    // Get the controller data:
    controller_data.throttle = map(elrs.getChannel(3), CONTROLLER_MIN, CONTROLLER_MAX, 1000, 2000);
    controller_data.roll = elrs.getChannel(1);
    controller_data.pitch = elrs.getChannel(2);
    controller_data.yaw = elrs.getChannel(4);
    controller_data.aux1 = elrs.getChannel(5);
    controller_data.aux2 = elrs.getChannel(6);
    controller_data.aux3 = elrs.getChannel(7);
    controller_data.aux4 = elrs.getChannel(8);

}

void controller_trheshold(){
    if ((controller_data.roll <= CONTROLL_THR_MAX) && (controller_data.roll >= CONTROLL_THR_MIN)){ controller_data.roll = 1500;}
    if ((controller_data.pitch <= CONTROLL_THR_MAX) && (controller_data.pitch >= CONTROLL_THR_MIN)){ controller_data.pitch = 1500;}
    if ((controller_data.yaw <= CONTROLL_THR_MAX) && (controller_data.yaw >= CONTROLL_THR_MIN)){ controller_data.yaw = 1500;}
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
    // These configurations are based on tables 44,45,47,48 in the datasheet https://www.pololu.com/file/0J1899/lsm6dso.pdf
    IMU.writeReg(LSM6::CTRL2_G, 0b01110000);  // 0b1010 for ODR 833 Hz, 0b0000 for 250 dps range. No internal filter
    IMU.writeReg(LSM6::CTRL1_XL, 0b01110000);  // 0b1010 for ODR 833 Hz, 0b0000 for 2g range. No internal filter.

    mag.enableDefault();
    mag.writeReg(LIS3MDL::CTRL_REG1, 0b11111010); // 1 KHz, high performance mode
    mag.writeReg(LIS3MDL::CTRL_REG2,0x10); // +- 4 gauss

}

void mapping_controller(char state){
    if (state == 's'){ // Mapping the controller input into desired angle:
        desired_attitude.roll = map(controller_data.roll, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_ANGLE, MAX_ANGLE);
        desired_attitude.pitch = map(controller_data.pitch, CONTROLLER_MIN, CONTROLLER_MAX, MAX_ANGLE, -MAX_ANGLE);
        desired_attitude.yaw = map(controller_data.yaw, CONTROLLER_MIN, CONTROLLER_MAX, MAX_ANGLE, -MAX_ANGLE);
    }
    else if (state == 'r'){ // Mapping the controller input into desired rate:
        desired_rate.roll = map(controller_data.roll, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_RATE, MAX_RATE);
        desired_rate.pitch = map(controller_data.pitch, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_RATE, MAX_RATE);
        desired_rate.yaw = map(controller_data.yaw, CONTROLLER_MIN, CONTROLLER_MAX, MAX_RATE, -MAX_RATE);
    }
}

void resetMicrocontroller() {
    // Trigger a software reset
    if (controller_data.aux3 > 1500) {
        SCB_AIRCR = 0x05FA0004;
        Serial.println(" reset ");
    }
}

void check_arming_state() {
    /// need to check it 
    // Using aux2 (channel 6) as the arming switch
    // You can change this to any aux channel you prefer
    if (controller_data.aux2 > 1500) {  // Switch is in high position
        if (THR_FS == true || controller_data.throttle < (MOTOR_START + 100)) {
            is_armed = true;
        }
    } else {  // Switch is in low position
        is_armed = false;
        THR_FS = false;
        motors.Disarm();  // Ensure motors are stopped when disarmed
        Reset_PID();      // Reset PID states when disarmed
        resetMicrocontroller();
    }
}


/********* Functions for filter choosing *********/
void compclass_function() {
    Comp_filter.UpdateQ(&meas, actual_dt / 2);
    Comp_filter.GetEulerRPYdeg(&estimated_attitude, meas.initial_heading);
    Comp_filter.GetQuaternion(&q_est);
}

void channel_estimated() {
    if (controller_data.aux4 > 1700) {
        filter_type = 0;
        // Serial.println("ekf");
    } else if (controller_data.aux4 < 1100) {
        filter_type = 1;
        // Serial.println("comclass");
    }
}

void filter_method() {
    channel_estimated();
    switch (filter_type) {
        case 0:  // compclass
            return compclass_function();
        case 1:  // ekkf
            return Ekf_filter.run_kalman(&estimated_attitude, &q_est);
        default:
            return Ekf_filter.run_kalman(&estimated_attitude, &q_est);
    }
}