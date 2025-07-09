#ifndef COMPCLASS_H
#define COMPCLASS_H

#include <Arduino.h>
#include <Var_types.h>

// Define PI if not already defined
#ifndef PI
#define PI 3.14159265358979323846
// Angle conversion constants:
#define rad2deg 180.0f/PI
#endif

// Motion thresholds and filter parameters
#define LOW_MOTION 0.15f * 9.81    // Slightly lower than 0.2f
#define HIGH_MOTION 1.0f * 9.81   // Lower than 1.5f to activate high correction sooner

#define HIGH_BETA 0.9f // Rely more on accelerometer
#define LOW_BETA 0.3f // Rely more on gyroscope
#define DEFAULT_BETA 0.8f


class CompFilter {
    public:
        CompFilter(bool _MAG = 1) : USE_MAG(_MAG) {}
        bool USE_MAG;


        quat_t q = {0.0, 0.0, 0.0, 1.0};
        quat_t qDot_prev ={0.0, 0.0, 0.0, 0.0};
        float accNorm = 0.0;
        float drift = 0.0;
        float driftRate = 0.005;
        float gravX , gravY, gravZ; // Unit vector in the direction of the estimated gravity


        void UpdateQ(vec3_t* gyro, vec3_t* acc, float dt);
        float calculateDynamicBeta(vec3_t acc);
        float invSqrt(float x);
        void GetQuaternion(quat_t* q_);
        void GetEulerRPYrad(attitude_s* , float);
        void GetEulerRPYdeg(attitude_s*);
        void estimatedGravityDir(float* , float* , float*);
        float GetAccZ(float , float , float );
};

#endif