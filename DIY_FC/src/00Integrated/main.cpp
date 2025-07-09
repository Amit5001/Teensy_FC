// Filter Library:
#include <01Filter/CompClass.h>
#include <01Filter/EkfClass.h>
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
#include <IMU_type.h>


// UDP Communication Libraries:
#include <00UDP/drone_comclass.h>


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
PID_const_t PID_CONSTS;
float t_PID_s = 0.0f;
float t_PID_r = 0.0f;
double actual_dt = 0.0f;

// Filters:
CompFilter Comp_filter(false); // True for enabling the magnetometer
// EKF Ekf_filter(&meas, 1/STAB_PERIOD);
EKF Ekf_filter(&meas, DT);
STD_Filter std_filter(&meas, DT); // Standard filter for the IMU data
int filter_type = 0; // 0 for Complementary, 1 for Kalman
Drone_com drone_com(&meas, &q_est, &desired_attitude, &motor_pwm, &desired_rate, &estimated_attitude, &estimated_rate, &PID_stab_out, &PID_rate_out, &controller_data, &PID_CONSTS);
IMU_Func IMU(&meas, SAMPLE_RATE); // IMU object for reading the IMU data

// Timers:
elapsedMicros motor_timer;
elapsedMicros stab_timer;
elapsedMicros stab_timer_filt;
elapsedMicros imu_timer;
elapsedMicros Data_timer;


/********************************************** Function Prototypes **********************************************/
void update_controller();
void mapping_controller(char);
void resetMicrocontroller();
void check_arming_state();
void controller_threshold();
void compclass_function();
void channel_estimated();
void filter_method();

/*********************************************** Main Code ***********************************************/

void setup() {
    // Initialize Serial Monitor:
    Serial.begin(115200);
    while (!Serial);
    Serial.println("USB Serial initialized");

    drone_com.init_com();

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
    IMU.init_IMU();
    // IMU_init();
    Serial.println("IMU initialized");

    // Initializing motors and PID:
    setPID_params(&PID_CONSTS);
    getbot_param(PID_CONSTS);
    IMU.Initial_Calibration();
    // GyroMagCalibration();
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
        // Update_Measurement();
        IMU.Read_IMU();
        // IMU.Read_IMU();
        Serial.println(meas.acc.z);
        
        if (stab_timer_filt >= STAB_PERIOD){
            filter_method();
            stab_timer_filt = 0;
        }

        
        if (is_armed){
            // Get Actual rates:
            estimated_rate.roll = meas.gyroDEG.x;
            estimated_rate.pitch = meas.gyroDEG.y;
            estimated_rate.yaw = meas.gyroDEG.z;

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
            drone_com.convert_Measurment_to_byte();
            drone_com.send_data();
            Data_timer = 0;
        }
        
        imu_timer = 0;
    }
}


/********************************************** Function Definitions **********************************************/

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

void controller_threshold(){
    if ((controller_data.roll <= CONTROLL_THR_MAX) && (controller_data.roll >= CONTROLL_THR_MIN)){ controller_data.roll = 1500;}
    if ((controller_data.pitch <= CONTROLL_THR_MAX) && (controller_data.pitch >= CONTROLL_THR_MIN)){ controller_data.pitch = 1500;}
    if ((controller_data.yaw <= CONTROLL_THR_MAX) && (controller_data.yaw >= CONTROLL_THR_MIN)){ controller_data.yaw = 1500;}
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
    Comp_filter.UpdateQ(&meas.gyroRAD, &meas.acc, DT);
    Comp_filter.GetEulerRPYdeg(&estimated_attitude);
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