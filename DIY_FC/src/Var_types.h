/*
Written by: Amit Gedj
Date: 15.10.24

Description: This file contains the definition of the different types used in the project.
              It also contains the definition of the different rates used in the project.
              The RATE_DO_EXECUTE macro is used to execute a certain code at a certain rate.
              The different rates are defined in the RATE_XXX_HZ macros.
              The different types are defined in the different structs.
              The different structs are:
                  - vec3_t: a 3D vector
                  - quat_t: a quaternion
                  - baro_t: a barometer measurement
                  - Measurement_t: a struct containing the different measurements
                  - flowMeasurement_t: a struct containing the flow measurements
                  - tofMeasurement_t: a struct containing the TOF measurements
                  - heightMeasurement_t: a struct containing the height measurements
                  - attitude_t: a struct containing the attitude angles
                  - state_t: a struct containing the state of the drone
                  - StabStep_t: a type used to count the number of stabilization steps
*/



#ifndef VAR_TYPES_H
#define VAR_TYPES_H

#include <Arduino.h>

// Frequencies to be used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define RATE_MAIN_LOOP RATE_1000_HZ
#define ATTITUDE_RATE RATE_500_HZ
#define POSITION_RATE RATE_100_HZ
#define RATE_HL_COMMANDER RATE_100_HZ
#define RATE_SUPERVISOR RATE_25_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

typedef struct{
    float x;
    float y;
    float z;
} vec3_t;


typedef struct{
    float x;
    float y;
    float z;
    float w;
} quat_t;

typedef struct baro_s {
  float pressure;           // mbar
  float temperature;        // degree Celcius
  float asl;                // m (ASL = altitude above sea level)
} baro_t;

typedef struct{
    vec3_t acc;
    vec3_t acc_bias = {0.0, 0.0, 0.0};
    vec3_t gyro;
    vec3_t gyro_bias = {0.0, 0.0, 0.0};
    float gyro_drift = {0.0};
    vec3_t mag;
    vec3_t mag_bias = {0.0, 0.0, 0.0};
    vec3_t initial_mag = {0.0, 0.0, 0.0};
    float initial_heading = 0.0;
    baro_t baro_data;
}Measurement_t;

typedef struct flowMeasurement_s {
  uint32_t timestamp;
  union {
    struct {
      float dpixelx;  // Accumulated pixel count x
      float dpixely;  // Accumulated pixel count y
    };
    float dpixel[2];  // Accumulated pixel count
  };
  float stdDevX;      // Measurement standard deviation
  float stdDevY;      // Measurement standard deviation
  float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;

/** LiDAR TOF measurement **/
typedef struct tofMeasurement_s {
  uint32_t timestamp;
  float distance;
  float stdDev;
} tofMeasurement_t;

/** Absolute height measurement **/
typedef struct heightMeasurement_s {
  uint32_t timestamp;
  float height;
  float stdDev;
} heightMeasurement_t;

typedef struct attitude_s{
    float roll;
    float pitch;
    float yaw;
}attitude_t;

// Specify addition operator for attitude_t
inline attitude_t operator+(const attitude_t& a,const attitude_t& b) {
    attitude_t result;
    result.roll = a.roll + b.roll;
    result.pitch = a.pitch + b.pitch;
    result.yaw = a.yaw + b.yaw;
    return result;
}

// Specify addition operator for attitude_t
inline attitude_t operator-(const attitude_t& a,const attitude_t& b) {
    attitude_t result;
    result.roll = a.roll - b.roll;
    result.pitch = a.pitch - b.pitch;
    result.yaw = a.yaw - b.yaw;
    return result;
}

// Specify the `+=` operator for attitude_t
inline attitude_t& operator+=(attitude_t& a, attitude_t& b) {
    a.roll += b.roll;
    a.pitch += b.pitch;
    a.yaw += b.yaw;
    return a;
}

// Specify the `-=` operator for attitude_t
inline attitude_t& operator-=(attitude_t& a, attitude_t& b) {
    a.roll -= b.roll;
    a.pitch -= b.pitch;
    a.yaw -= b.yaw;
    return a;
}

typedef struct state_s{
    attitude_t attitude_angles;
    quat_t attitudeQuaternion;
    float height;
    vec3_t velocity;
    vec3_t position;
    vec3_t acceleration;
}state_t;

typedef struct PID_Params_s{
    float RollP;
    float RollI;
    float RollD;
    float PitchP;
    float PitchI;
    float PitchD;
    float YawP;
    float YawI;
    float YawD;

    float Imax_roll;
    float Imax_pitch;
    float Imax_yaw;

}PID_Params_t;

typedef struct PID_out_s{
    attitude_t P_term;
    attitude_t I_term;
    attitude_t D_term;
    attitude_t PID_ret = {0.0, 0.0, 0.0};
    attitude_t prev_err = {0.0, 0.0, 0.0};
    attitude_t prev_Iterm = {0.0, 0.0, 0.0};

}PID_out_t;

typedef struct motor_s{
    int M1_pin;
    int M2_pin;
    int M3_pin;
    int M4_pin;
    int PWM1;
    int PWM2;
    int PWM3;
    int PWM4;
}motor_t;

typedef struct Controller_s{
    int throttle;
    int roll;
    int pitch;
    int yaw;
    int aux1;
    int aux2;
    int aux3;
    int aux4;
}Controller_t; 


#endif