#ifndef MEKF_H
#define MEKF_H

#include <Arduino.h>
#include <Var_types.h>

typedef enum {
    QUAT_W,
    QUAT_X,
    QUAT_Y,
    QUAT_Z,
    GYRO_BIAS_X,
    GYRO_BIAS_Y,
    GYRO_BIAS_Z,
    NUM_STATES
} StateIDX_t;

#define IMU_timer 1000 // IMU timer in Hz
#define WALK_SIGMA 2.036*(1e-5f)  // rad/sqrt(s) - Walk standard deviation

typedef struct KalmanParam_s{
    quat_t InitialQuaternion;
    vec3_t InitialGyroBias;
    float Pgain;
    float Qgain;
    float Rgain;
    float R[3][3];
    float P[NUM_STATES][NUM_STATES];
    float Q[NUM_STATES][NUM_STATES];
    float R[NUM_STATES][NUM_STATES];

} KalmanParam_t;


#endif