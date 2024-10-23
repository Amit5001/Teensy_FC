/*

This Filter is based on crazyflie's kalman filter implementation. It is a 9 state kalman filter that estimates position, velocity, acceleration, and attitude. It is based on the following paper:
Their main filter for attitude estimation is the Madgwick filter, which is a 4 state filter that estimates the quaternion. The kalman filter is used to estimate the position, velocity, and acceleration.
    https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/interface/kalman_core/kalman_core.h#L150
    https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/kalman_core/kalman_core.c#L574
*/



#ifndef ESTIMATE_KALMAN_H
#define ESTIMATE_KALMAN_H

#include <Arduino.h>
#include <Var_types.h>

typedef enum
{
  KC_STATE_X, KC_STATE_Y, KC_STATE_Z, KC_STATE_PX, KC_STATE_PY, KC_STATE_PZ, KC_STATE_D0, KC_STATE_D1, KC_STATE_D2, KC_STATE_DIM
} kalmanCoreStateIdx_t;

typedef struct {
    uint32_t count;
    uint32_t expectedMin;
    uint32_t expectedMax;
    uint32_t nextEvaluationTimeMs;
    uint32_t evaluationIntervalMs;
    uint32_t latestCount;
    uint8_t skip;
} rateSupervisor_t;

typedef struct {
    float S[KC_STATE_DIM]; // state
    quat_t q; // quaternion
    vec3_t gyroBias; // gyro bias
    float R[3][3]; // rotation matrix
    quat_t initialQuaternion; // initial quaternion
    vec3_t initialGyroBias; // initial gyro bias

    float P[KC_STATE_DIM][KC_STATE_DIM]; // covariance matrix

    bool isUpdated;
    uint32_t lastPredictTimeMs;
    uint32_t lastProcessNoiseUpdateMs;

}KalmanCoreData_t;

typedef struct{
    // Initial variances, uncertain of position, but know we're stationary and roughly flat
    float stdDevInitialPosition_xy;
    float stdDevInitialPosition_z;
    float stdDevInitialVelocity;
    float stdDevInitialAttitude_rollpitch;
    float stdDevInitialAttitude_yaw;

    float procNoiseAcc_xy;
    float procNoiseAcc_z;
    float procNoiseVel;
    float procNoisePos;
    float procNoiseAtt;
    float measNoiseBaro;           // meters
    float measNoiseGyro_rollpitch; // radians per second
    float measNoiseGyro_yaw;       // radians per second

    float initialX;
    float initialY;
    float initialZ;

    float initialRoll;
    float initialPitch;
    float initialYaw;

    // Initial yaw in radians.
    // 0 --- facing positive X
    // PI / 2 --- facing positive Y
    // PI --- facing negative X
    // 3 * PI / 2 --- facing negative Y
    float initialYaw;


}KalmanCoreParams_t;


void estimatorKalmanInit(void);
bool estimatorKalmanTest(void);
void estimatorKalman(state_t *state, const StabStep_t stabilizerStep);

void estimatorKalmanTaskInit();
bool estimatorKalmanTaskTest();

void estimatorKalmanGetEstimatedPos(vec3_t* pos);

/**
 * Copies 9 floats representing the current state rotation matrix
 */
void estimatorKalmanGetEstimatedRot(float * rotationMatrix);

#endif