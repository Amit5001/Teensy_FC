#ifndef COMPCLASS_H
#define COMPCLASS_H

#include <Arduino.h>
#include <Var_types.h>

// Define PI if not already defined
#ifndef PI
#define PI 3.14159265358979323846
#endif

// Angle conversion constants:
#define rad2deg 180.0f/PI

// Motion thresholds and filter parameters
// #define LOW_MOTION 0.2f
// #define HIGH_MOTION 1.5f
#define LOW_MOTION 0.15f    // Slightly lower than 0.2f
#define HIGH_MOTION 1.5f    // Lower than 1.5f to activate high correction sooner

#define HIGH_BETA 0.5f // Rely more on accelerometer
#define LOW_BETA 0.05f // Rely more on gyroscope
#define DEFAULT_BETA 0.5f

// Filter Frequencies:
#define ACC_LPF_FREQ 20.0f   // Increase from 10.0f
#define GYRO_LPF_FREQ 70.0f  // Increase from 40.0f
#define GYRO_HPF_FREQ 1.0f   // Lower from 2.5f to reduce drift removal aggressiveness
#define MAG_LPF_FREQ 15.0f   // Increase from 10.0f

// Calculate filter coefficients based on cutoff frequencies
// static const float SAMPLE_RATE = 1100.0f;
// static const float DT = 1.0f/SAMPLE_RATE;
static const float ALPHA_ACC_LPF = (2.0f * PI * ACC_LPF_FREQ * DT / (2.0f * PI * ACC_LPF_FREQ * DT + 1.0f));
static const float ALPHA_GYRO_LPF = (2.0f * PI * GYRO_LPF_FREQ * DT / (2.0f * PI * GYRO_LPF_FREQ * DT + 1.0f));
static const float ALPHA_HPF = (1.0f / (2.0f * PI * GYRO_HPF_FREQ * DT + 1.0f));
static const float ALPHA_MAG_LPF = (2.0f * PI * MAG_LPF_FREQ * DT / (2.0f * PI * MAG_LPF_FREQ * DT + 1.0f));

// Motor and filter frequencies
#define MOTOR_FREQ 600.0f
#define NOTCH_BW 30.0f
#define ACC_LPF_CUTOFF 10.0f
#define GYRO_LPF_CUTOFF 50.0f

class CompFilter {
    public:
        CompFilter(bool _MAG = 1) : USE_MAG(_MAG) {}
        bool USE_MAG;


        quat_t q = {0.0, 0.0, 0.0, 1.0};
        // Params for HPF and LPF:
        vec3_t accFiltered = {0.0, 0.0, 0.0};
        vec3_t gyroFiltered = {0.0, 0.0, 0.0};
        vec3_t magFiltered = {0.0, 0.0, 0.0};
        float gyroNorm = 0.0;
        float drift = 0.0;
        float driftRate = 0.005;
        float gravX , gravY, gravZ; // Unit vector in the direction of the estimated gravity


        void UpdateQ(Measurement_t* , float );
        void InitialFiltering(Measurement_t* );
        float calculateDynamicBeta(Measurement_t );
        float invSqrt(float x);
        void GetQuaternion(quat_t* q_);
        void GetEulerRPYrad(attitude_s* , float);
        void GetEulerRPYdeg(attitude_s* , float);
        void estimatedGravityDir(float* , float* , float*);
        float GetAccZ(float , float , float );
};

#endif