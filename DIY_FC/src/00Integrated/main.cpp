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
#include <00UDP/Drone_com.h>


/********************************************** Definitions **********************************************/

#define ELRS_SERIAL 1
#define MOTOR1_PIN 2
#define MOTOR2_PIN 3
#define MOTOR3_PIN 4
#define MOTOR4_PIN 5
// #define ESC_FREQUENCY 500


/**** Max Angle and max rate ****/
#define MAX_ANGLE 15.0f
#define MAX_RATE 500.0f
#define CONTROLLER_MIN 988
#define CONTROLLER_MAX 2012


/**** IMU Data parameters:  ****/
#define POL_GYRO_SENS 17.5/1000.0f    // FS = 500
#define POL_ACC_SENS 0.061/1000.0f     // FS = 2g, 0.061 mg/LSB
#define POL_MAG_SENS 1/6842.0f         // FS = 4 gauss

#define IMU_THRESHOLD 0.005f



/**** Unit Conversion parameters: ****/
#define PI 3.14159265358979323846f
#define rad2deg 180.0f/PI
#define deg2rad PI/180.0f


/**** UDP Communication: ****/
float* imu_data = (float*)calloc(6, sizeof(float));
uint8_t imu_byte[sizeof(float)*6];
float* mag_data = (float*)calloc(3, sizeof(float));
uint8_t mag_byte[sizeof(float)*3];
float* euler_data = (float*)calloc(3, sizeof(float));
uint8_t euler_byte[sizeof(float)*3];
float* quaternion_data = (float*)calloc(4, sizeof(float));
uint8_t quaternion_byte[sizeof(float)*4];
int* rc_ch_data = (int*)calloc(16, sizeof(int));
uint8_t rc_byte[sizeof(int)*16];
float* desired_rate_data = (float*)calloc(3, sizeof(float));
uint8_t desired_rate_byte[sizeof(float)*3];
float* desired_attitude_data = (float*)calloc(3, sizeof(float));
uint8_t desired_attitude_byte[sizeof(float)*3];
float* estimated_attitude_data = (float*)calloc(3, sizeof(float));
uint8_t estimated_attitude_byte[sizeof(float)*3];
float* estimated_rate_data = (float*)calloc(3, sizeof(float));
uint8_t estimated_rate_byte[sizeof(float)*3];
float* PID_stab_out_data = (float*)calloc(12, sizeof(float));
uint8_t PID_stab_out_byte[sizeof(float)*12];
float* PID_rate_data = (float*)calloc(12, sizeof(float));
uint8_t PID_rate_byte[sizeof(float)*12];
float* motor_data = (float*)calloc(4, sizeof(float));
uint8_t motor_pwm_byte[sizeof(float)*4];

#define PP Serial.println("")
#define BUF Serial.print("//")
#define MPU_TICK_RATE 10000

constexpr uint8_t IP_ADDRESS[4] = {192, 169, 1, 199};
constexpr uint16_t PORT_NUMBER = 8888;
const SocketAddress SOCKET_ADDRESS = SocketAddress(IP_ADDRESS, PORT_NUMBER);
const SocketAddress otherAddress = SocketAddress(IPAddress(192, 169, 1, 10), 12000);
RTCom rtcomSocket(SOCKET_ADDRESS, RTComConfig(1, 100, 200, 500));
RTComSession *rtcomSession = nullptr;




#define elrsSerial Serial1  // Use Serial1 for the CRSF communication


/********************************************** Variables Decleration **********************************************/

// ELRS Controller:
AlfredoCRSF elrs;
Controller_s controller_data;


// Motors Variables:
Motors motors(MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN);
motor_t motor_pwm;
const unsigned long PWM_PERIOD_1 = 1000000 / ESC_FREQUENCY; // 1,000,000 us / frequency in Hz. Recieving PWM signal every 2ms
bool is_armed = false;

// IMU and Filter Variables:
LSM6 IMU;
LIS3MDL mag;
LPS baro;


// Measurement struct
Measurement_t meas;
quat_t q_est;
vec3_t euler_angles;

// Filter Object:
CompFilter Pololu_filter(false); // True for enabling the magnetometer
// float dt = 1/1100.0f; // 1.1kHz sample rate in seconds

// Desired Attitude - From the controller:
attitude_t desired_attitude;
int mid_value = 1500;
attitude_t motor_input; // Currently not in use. replaced by PID_rate_out.PID_ret
attitude_t desired_rate;
attitude_t estimated_attitude;
attitude_t estimated_rate;
PID_out_t PID_stab_out;
PID_out_t PID_rate_out;
const unsigned long STAB_PERIOD = 1000000 / (ESC_FREQUENCY/2); // 300 Hz period in microseconds
float t_PID_s = 0.0f;
float t_PID_r = 0.0f;

// Timers:
elapsedMicros motor_timer;
elapsedMicros stab_timer;
elapsedMicros imu_timer;


/********************************************** Function Prototypes **********************************************/
void Update_Measurement();
void GyroMagCalibration();
void update_controller();
void IMU_init();
void mapping_controller(char);
void resetMicrocontroller();
void check_arming_state();
void onConnection(RTComSession &session);
void convert_Measurment_to_byte();
void emit_data();

/********************************************** Main Code **********************************************/

void setup() {
    // Initialize Serial Monitor:
    Serial.begin(115200);
    while (!Serial);
    Serial.println("USB Serial initialized");

    rtcomSocket.begin();
    rtcomSocket.onConnection(onConnection);
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

    // Update the measurement:
    Update_Measurement();

    // Update the quaternion:
    float actual_dt = (float)imu_timer / 1000000.0f;
    imu_timer = 0;
    Pololu_filter.UpdateQ(&meas, actual_dt/2);
    // Get the Euler angles:
    Pololu_filter.GetEulerRPYdeg(&estimated_attitude, meas.initial_heading);
    // Get the quaternion:
    Pololu_filter.GetQuaternion(&q_est);
    
    //**************** DEBUGGING STARTS HERE */
    Serial.println(estimated_attitude.roll);

    if (is_armed){
        // Get Actual rates:
        estimated_rate.roll = meas.gyro_LPF.x * rad2deg;
        estimated_rate.pitch = meas.gyro_LPF.y * rad2deg;
        estimated_rate.yaw = meas.gyro_LPF.z * rad2deg;

        if ((controller_data.aux1 > 1500) && (stab_timer >= STAB_PERIOD)){ // Stabilize mode:
                // Calculating dt for the PID- in seconds:
                t_PID_s = (float)stab_timer / 1000000.0f;
                mapping_controller('s');
                PID_stab_out = PID_stab(desired_attitude, estimated_attitude, t_PID_s);
                desired_rate = PID_stab_out.PID_ret;
                stab_timer = 0;
        }
        else if (controller_data.aux1 < 1500) { // Acro mode:
            mapping_controller('r');
        }

        // Set the motor PWM:
        if ((controller_data.throttle > 1000) && (motor_timer >= PWM_PERIOD_1)){
            t_PID_r = (float)motor_timer / 1000000.0f;
            PID_rate_out = PID_rate(desired_rate, estimated_rate, t_PID_r);
            motors.Motor_Mix(PID_rate_out.PID_ret, controller_data.throttle);
            motors.set_motorPWM();
            motor_timer = 0;

        }
    
        //Getting the motors struct to send data back:
        motor_pwm = motors.Get_motor();

        // Sending new UDP Packet:
        convert_Measurment_to_byte();
        rtcomSocket.process();
        if (rtcomSocket.isSessionConnected(rtcomSession)){
            emit_data();};
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
    if (abs(meas.acc.x) < IMU_THRESHOLD) { meas.acc.x = 0;}
    if (abs(meas.acc.y) < IMU_THRESHOLD) { meas.acc.y = 0;}
    if (abs(meas.acc.z) < IMU_THRESHOLD) { meas.acc.z = 0;}

    meas.gyro.x = IMU.g.x * POL_GYRO_SENS * deg2rad-meas.gyro_bias.x;
    meas.gyro.y = IMU.g.y * POL_GYRO_SENS * deg2rad-meas.gyro_bias.y;
    meas.gyro.z = IMU.g.z * POL_GYRO_SENS * deg2rad-meas.gyro_bias.z;
    if (abs(meas.gyro.x) < IMU_THRESHOLD) { meas.gyro.x = 0;}
    if (abs(meas.gyro.y) < IMU_THRESHOLD) { meas.gyro.y = 0;}
    if (abs(meas.gyro.z) < IMU_THRESHOLD) { meas.gyro.z = 0;}

    meas.mag.x = mag.m.x * POL_MAG_SENS - meas.mag_bias.x;
    meas.mag.y = mag.m.y * POL_MAG_SENS - meas.mag_bias.y;
    meas.mag.z = mag.m.z * POL_MAG_SENS - meas.mag_bias.z;
    if (abs(meas.mag.x < IMU_THRESHOLD)) { meas.mag.x = 0;}
    if (abs(meas.mag.y < IMU_THRESHOLD)) { meas.mag.y = 0;}
    if (abs(meas.mag.z < IMU_THRESHOLD)) { meas.mag.z = 0;}

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
    controller_data.throttle = elrs.getChannel(3);
    controller_data.roll = elrs.getChannel(1);
    controller_data.pitch = elrs.getChannel(2);
    controller_data.yaw = elrs.getChannel(4);
    controller_data.aux1 = elrs.getChannel(5);
    controller_data.aux2 = elrs.getChannel(6);
    controller_data.aux3 = elrs.getChannel(7);
    controller_data.aux4 = elrs.getChannel(8);
    if (controller_data.throttle > 1800){controller_data.throttle = 1800;}

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
    IMU.writeReg(LSM6::CTRL2_G, 0b10000100);  // 0b1010 for ODR 1.66 kHz, 0b0000 for 500 dps range
    IMU.writeReg(LSM6::CTRL1_XL, 0b10000010);  // 0b1010 for ODR 1.66 kHz, 0b0000 for 2g range with  ODR/10 filter
     // 3. Enable additional filtering (LPF2) for gyroscope with medium cutoff
     IMU.writeReg(LSM6::CTRL6_C, 0b00001001); 

    mag.enableDefault();
    mag.writeReg(LIS3MDL::CTRL_REG1, 0b11111010); // 1 KHz, high performance mode
    mag.writeReg(LIS3MDL::CTRL_REG2,0x10); // +- 4 gauss

    // Set high-performance mode for accelerometer
    IMU.writeReg(LSM6::CTRL6_C, 0x00); 

    // Set high-performance mode for gyroscope
    IMU.writeReg(LSM6::CTRL7_G, 0x00);
}

void mapping_controller(char state){
    if (state == 's'){ // Mapping the controller input into desired angle:
        desired_attitude.roll = map(controller_data.roll, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_ANGLE, MAX_ANGLE);
        desired_attitude.pitch = map(controller_data.pitch, CONTROLLER_MIN, CONTROLLER_MAX, -MAX_ANGLE, MAX_ANGLE);
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
        if (is_armed == true || controller_data.throttle < (MOTOR_START + 100)) {
            is_armed = true;
            is_armed == true;
        }
    } else {  // Switch is in low position
        is_armed = false;
        is_armed == false;
        motors.Disarm();  // Ensure motors are stopped when disarmed
        Reset_PID();      // Reset PID states when disarmed
        resetMicrocontroller();
    }
}

// *********** UDP Communication Functions *********** //

#define MAG 'm'
#define P_IMU 'p'
#define EUILER 'e'
#define RC 'r'
#define Quaternion 'q'
#define D_RATE 'z'
#define D_ATI 'w'
#define MOTOR_DATA_CONST 'a'
#define EST_RATE 'n'
#define PID_stab_prase 'l'
#define PID_rate_prase 'b'


void onConnection(RTComSession &session) {
    Serial.printf("Session created with %s\r\n", session.address.toString());
    rtcomSession = &session;

    session.onReceive([](const uint8_t *bytes, size_t size) {
        char data[size + 1] = {0};
        memcpy(data, bytes, size);
        Serial.println(data);
    });
    session.onDisconnect([&session]() {
        Serial.print("Disconnected session: ");
        Serial.println(session.address.toString());
    });
}

void convert_Measurment_to_byte(){

    imu_data[0] = meas.acc.x;
    imu_data[1] = meas.acc.y;
    imu_data[2] =  meas.acc.z;
    imu_data[3] = meas.gyro.x;
    imu_data[4] = meas.gyro.y;
    imu_data[5] = meas.gyro.z;
    memcpy(imu_byte, imu_data, sizeof(imu_byte));

    mag_data[0] = meas.mag.x;
    mag_data[1] = meas.mag.y;
    mag_data[2] = meas.mag.z;
    memcpy(mag_byte, mag_data, sizeof(mag_byte));

    quaternion_data[0] = q_est.w;
    quaternion_data[1] = q_est.x;
    quaternion_data[2] = q_est.y;
    quaternion_data[3] = q_est.z;
    memcpy(quaternion_byte, quaternion_data, sizeof(quaternion_byte));


    desired_attitude_data[0] = desired_attitude.pitch;
    desired_attitude_data[1] = desired_attitude.roll;
    desired_attitude_data[2] = desired_attitude.yaw;
    memcpy(desired_attitude_byte, desired_attitude_data, sizeof(desired_attitude_byte));


    euler_data[0] = estimated_attitude.pitch;
    euler_data[1] = estimated_attitude.roll;
    euler_data[2] = estimated_attitude.yaw;
    memcpy(euler_byte, euler_data, sizeof(euler_byte));

    desired_rate_data[0] = desired_rate.pitch;
    desired_rate_data[1] = desired_rate.roll;
    desired_rate_data[2] = desired_rate.yaw;
    memcpy(desired_rate_byte, desired_rate_data, sizeof(desired_rate_byte));

    estimated_rate_data[0] = estimated_rate.pitch;
    estimated_rate_data[1] = estimated_rate.roll;
    estimated_rate_data[2] = estimated_rate.yaw;
    memcpy(estimated_rate_byte, estimated_rate_data, sizeof(estimated_rate_byte));
    PID_stab_out_data[0] = PID_stab_out.P_term.pitch;
    PID_stab_out_data[1] = PID_stab_out.P_term.roll;
    PID_stab_out_data[2] = PID_stab_out.P_term.yaw;
    PID_stab_out_data[3] = PID_stab_out.I_term.pitch;
    PID_stab_out_data[4] = PID_stab_out.I_term.roll;
    PID_stab_out_data[5] = PID_stab_out.I_term.yaw;
    PID_stab_out_data[6] = PID_stab_out.D_term.pitch;
    PID_stab_out_data[7] = PID_stab_out.D_term.roll;
    PID_stab_out_data[8] = PID_stab_out.D_term.yaw;
    PID_stab_out_data[9] = PID_stab_out.PID_ret.pitch;
    PID_stab_out_data[10] = PID_stab_out.PID_ret.roll;
    PID_stab_out_data[11] = PID_stab_out.PID_ret.yaw;
    memcpy(PID_stab_out_byte, PID_stab_out_data, sizeof(PID_stab_out_byte));

    PID_rate_data[0] = PID_rate_out.P_term.pitch;
    PID_rate_data[1] = PID_rate_out.P_term.roll;
    PID_rate_data[2] = PID_rate_out.P_term.yaw;
    PID_rate_data[3] = PID_rate_out.I_term.pitch;
    PID_rate_data[4] = PID_rate_out.I_term.roll;
    PID_rate_data[5] = PID_rate_out.I_term.yaw;
    PID_rate_data[6] = PID_rate_out.D_term.pitch;
    PID_rate_data[7] = PID_rate_out.D_term.roll;
    PID_rate_data[8] = PID_rate_out.D_term.yaw;
    PID_rate_data[9] = PID_rate_out.PID_ret.pitch;
    PID_rate_data[10] = PID_rate_out.PID_ret.roll;
    PID_rate_data[11] = PID_rate_out.PID_ret.yaw;
    memcpy(PID_rate_byte, PID_rate_data, sizeof(PID_rate_byte));

    motor_data[0]=motor_pwm.PWM1;
    motor_data[1]=motor_pwm.PWM2;
    motor_data[2]=motor_pwm.PWM3;
    motor_data[3]=motor_pwm.PWM4;
    for (size_t i = 0; i < 4; i++){motor_data[i] = map(motor_data[i],2200,4000,0,100);}
    memcpy(motor_pwm_byte, motor_data, sizeof(motor_pwm_byte));
}

    void emit_data(){
        rtcomSession->emitTyped(imu_byte,sizeof(imu_byte),P_IMU);
        rtcomSession->emitTyped(mag_byte,sizeof(mag_byte),MAG);
        rtcomSession->emitTyped(quaternion_byte,sizeof(quaternion_byte),Quaternion);
        rtcomSession->emitTyped(euler_byte,sizeof(euler_byte),EUILER);
        rtcomSession->emitTyped(rc_byte,sizeof(rc_byte),RC);
        rtcomSession->emitTyped(desired_rate_byte,sizeof(desired_rate_byte),D_RATE);
        rtcomSession->emitTyped(desired_attitude_byte,sizeof(desired_attitude_byte),D_ATI);
        rtcomSession->emitTyped(estimated_rate_byte,sizeof(estimated_rate_byte),EST_RATE);
        rtcomSession->emitTyped(PID_stab_out_byte,sizeof(PID_stab_out_byte),PID_stab_prase);
        rtcomSession->emitTyped(PID_rate_byte,sizeof(PID_rate_byte),PID_rate_prase);
        rtcomSession->emitTyped(motor_pwm_byte,sizeof(motor_pwm_byte),MOTOR_DATA_CONST);
}





/*
------------------------------------------ UDP Functions -end -----------------------------------------
*/
