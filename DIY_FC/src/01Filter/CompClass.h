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
// #define LOW_MOTION 0.001*125.0f
// #define HIGH_MOTION 0.008*125.0f
#define LOW_MOTION 0.2f
#define HIGH_MOTION 1.5f
// #define HIGH_BETA 0.2f
// #define LOW_BETA 0.03f
#define HIGH_BETA 0.1f
#define LOW_BETA 0.02f
#define DEFAULT_BETA 0.05f

// Filter Frequencies:
#define ACC_LPF_FREQ 10.0f   // 10 Hz cutoff for accelerometer
#define GYRO_LPF_FREQ 50.0f  // 50 Hz cutoff for gyro
#define GYRO_HPF_FREQ 0.1f   // 0.1 Hz cutoff for gyro drift removal
#define MAG_LPF_FREQ 10.0f   // 10 Hz cutoff for magnetometer

// Calculate filter coefficients based on cutoff frequencies
static const float SAMPLE_RATE = 1100.0f;
static const float DT = 1.0f/SAMPLE_RATE;
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