#include <Arduino.h>
#include <01Filter/ComplimentaryFilter/CompFilter.h>
#include <Var_types.h>

#define PI 3.14159265358979323846f
#define rad2deg 180.0f/PI
#define deg2rad PI/180.0f

#define USE_MAG 1


// Adaptive Beta parameters, LPF and HPF:
#define LOW_MOTION 0.001*245.0f
#define HIGH_MOTION 0.008*245.0f
#define HIGH_BETA 0.8f
#define LOW_BETA 0.01f
#define DEFAULT_BETA 0.04f
#define ALPHA_LPF 0.25f
#define ALPHA_HPF 0.75f

quat_t q = {0.0, 0.0, 0.0, 1.0};

// Params for HPF and LPF:
vec3_t accFiltered = {0.0, 0.0, 0.0};
vec3_t gyroFiltered = {0.0, 0.0, 0.0};
vec3_t magFiltered = {0.0, 0.0, 0.0};
float gyroNorm = 0.0;
float drift = 0.0;
float driftRate = 0.005;


static float gravX , gravY, gravZ; // Unit vector in the direction of the estimated gravity

//static float baseZaccel = 1.0; // Base acceleration in the Z direction

void UpdateQ(Measurement_t* meas, float dt){
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy, _8qx, _8qy, qwqw, qxqx, qyqy, qzqz;

    // Initial Filtering
    InitialFiltering(meas);


    // Model based time propagation
    qDot1 = 0.5f * (-q.x * meas->gyro.x - q.y * meas->gyro.y - q.z * meas->gyro.z);
    qDot2 = 0.5f * (q.w * meas->gyro.x + q.y * meas->gyro.z - q.z * meas->gyro.y);
    qDot3 = 0.5f * (q.w * meas->gyro.y - q.x * meas->gyro.z + q.z * meas->gyro.x);
    qDot4 = 0.5f * (q.w * meas->gyro.z + q.x * meas->gyro.y - q.y * meas->gyro.x);
    
    float BETA = calculateDynamicBeta(*meas);
    //float BETA = DEFAULT_BETA;

    if(!(meas->acc.x == 0.0 && meas->acc.y ==0.0 && meas->acc.z ==0.0)) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(meas->acc.x * meas->acc.x + meas->acc.y * meas->acc.y + meas->acc.z * meas->acc.z);
        meas->acc.x *= recipNorm;
        meas->acc.y *= recipNorm;
        meas->acc.z *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2qw = 2.0f * q.w;
        _2qx = 2.0f * q.x;
        _2qy = 2.0f * q.y;
        _2qz = 2.0f * q.z;
        _4qw = 4.0f * q.w;
        _4qx = 4.0f * q.x;
        _4qy = 4.0f * q.y;
        _8qx = 8.0f * q.x;
        _8qy = 8.0f * q.y;
        qwqw = q.w * q.w;
        qxqx = q.x * q.x;
        qyqy = q.y * q.y;
        qzqz = q.z * q.z;

        // Gradient decent algorithm corrective step
        s0 = _4qw * qyqy + _2qy * meas->acc.x + _4qw * qxqx - _2qx * meas->acc.y;
        s1 = _4qx * qzqz - _2qz * meas->acc.x + 4.0f * qwqw * q.x - _2qw * meas->acc.y - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * meas->acc.z;
        s2 = 4.0f * qwqw * q.y + _2qw * meas->acc.x + _4qy * qzqz - _2qz * meas->acc.y - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * meas->acc.z;
        s3 = 4.0f * qxqx * q.z - _2qx * meas->acc.x + 4.0f * qyqy * q.z - _2qy * meas->acc.y;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;


        // Apply feedback step
        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;


    }
    if(USE_MAG &&( meas->mag.x != 0.0 && meas->mag.y != 0.0 && meas->mag.z != 0.0) && (gyroNorm > HIGH_MOTION)){
    //if(USE_MAG &&( meas->mag.x != 0.0 && meas->mag.y != 0.0 && meas->mag.z != 0.0)){
        float yawMag = atan2f(meas->mag.y, meas->mag.x);
        float yawError = yawMag - atan2f(2*(q.w*q.z + q.x*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
        // if (yawError > PI) yawError -= 2*PI;
        // if (yawError < -PI) yawError += 2*PI;
        if(yawError > 0.1*deg2rad) BETA = HIGH_BETA;
        BETA = DEFAULT_BETA;

        // Apply feedback step (Magnetometer) Only for Yaw related parameters
        qDot1 -= BETA * yawError;
        // qDot2 -= BETA * yawError;
        // qDot3 -= BETA * yawError;
        qDot4 -= BETA * yawError;

    }


    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * dt;
    q.x += qDot2 * dt;
    q.y += qDot3 * dt;
    q.z += qDot4 * dt;

    // Normalise quaternion in order to get unit length quaternion
    recipNorm = invSqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    estimatedGravityDir(&gravX, &gravY, &gravZ);

}



// Returning the estimated quaternion
void GetQuaternion(quat_t* q_){
    q_->x = q.x;
    q_->y = q.y;
    q_->z = q.z;
    q_->w = q.w;

}

void GetEulerRPYrad(vec3_t* rpy){
    float gx = gravX;
    float gy = gravY;
    float gz = gravZ;

    if (gx > 1) gx = 1;
    if (gx < -1) gx = -1;

    // Currently returend in radians, can be converted to degrees by multiplying by rad2deg
    rpy->z = atan2f(2*(q.w*q.z + q.x*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    rpy->y = asinf(gx);
    rpy->x = atan2f(gy, gz);
}

void GetEulerRPYdeg(vec3_t* rpy, float initial_heading){
    float gx = gravX;
    float gy = gravY;
    float gz = gravZ;
    if (gx > 1) gx = 1;
    if (gx < -1) gx = -1;

    // Currently returend in radians, can be converted to degrees by multiplying by rad2deg
    rpy->z = atan2f(2*(q.w*q.z + q.x*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z) * rad2deg;
        if (rpy->z < 0) {
        rpy->z += 360.0f;
    }
    //rpy->y = asinf(gx) * rad2deg;
    rpy->y = atan2f(2 * (q.w * q.y - q.x * q.z), 1 - 2 * (q.y * q.y + q.z * q.z)) * rad2deg;
    rpy->x = atan2f(gy, gz) * rad2deg;
}

//---------------------------------------------------------------------------------------------------
// Inverse square root function:
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float GetAccZ(float ax, float ay, float az) {
    return (ax*gravX + ay*gravY + az*gravZ);
}

void estimatedGravityDir(float* gx, float* gy, float*gz){
    *gx = 2*(q.x*q.z - q.y*q.w);
    *gy = 2*(q.y*q.z + q.x*q.w);
    *gz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}
float calculateDynamicBeta(Measurement_t meas) {
    // Compute the norm (magnitude) of the gyroscope vector
    gyroNorm = sqrtf(meas.gyro.x * meas.gyro.x + meas.gyro.y * meas.gyro.y + meas.gyro.z * meas.gyro.z);

    // Adapt Beta based on gyroscope norm
    if (gyroNorm < LOW_MOTION) {
        // System is likely stable or slow-moving, increase Beta for more correction
        // Serial.println("Low Motion");
        return HIGH_BETA;
    } else if (gyroNorm > HIGH_MOTION) {
        // System is moving fast, reduce Beta to rely more on gyroscope
        // Serial.println("High Motion");
        return LOW_BETA;
    } else {
        // Default case, normal correction
        // Serial.println("Default Motion");
        return DEFAULT_BETA;
    }
}


void InitialFiltering(Measurement_t* meas){
     // Apply Low-pass Filter to Accel
    accFiltered.x = (1 - ALPHA_LPF) * accFiltered.x + ALPHA_LPF * meas->acc.x;
    accFiltered.y = (1 - ALPHA_LPF) * accFiltered.y + ALPHA_LPF * meas->acc.y;
    accFiltered.z = (1 - ALPHA_LPF) * accFiltered.z + ALPHA_LPF * meas->acc.z;
    meas->acc.x = accFiltered.x;
    meas->acc.y = accFiltered.y;
    meas->acc.z = accFiltered.z;

    // Apply High-pass Filter to Gyro
    static vec3_t gyroPrev = {0.0, 0.0, 0.0};
    gyroFiltered.x = ALPHA_HPF * (gyroFiltered.x + meas->gyro.x - gyroPrev.x);
    gyroFiltered.y = ALPHA_HPF * (gyroFiltered.y + meas->gyro.y - gyroPrev.y);
    gyroFiltered.z = ALPHA_HPF * (gyroFiltered.z + meas->gyro.z - gyroPrev.z);
    meas->gyro.x = gyroFiltered.x;
    meas->gyro.y = gyroFiltered.y;
    meas->gyro.z = gyroFiltered.z;
    gyroPrev.x = meas->gyro.x;
    gyroPrev.y = meas->gyro.y;
    gyroPrev.z = meas->gyro.z;

    if (USE_MAG){
        // Apply Low-pass Filter to Mag
        magFiltered.x = (1 - ALPHA_LPF) * magFiltered.x + ALPHA_LPF * meas->mag.x;
        magFiltered.y = (1 - ALPHA_LPF) * magFiltered.y + ALPHA_LPF * meas->mag.y;
        magFiltered.z = (1 - ALPHA_LPF) * magFiltered.z + ALPHA_LPF * meas->mag.z;
        meas->mag.x = magFiltered.x;
        meas->mag.y = magFiltered.y;
        meas->mag.z = magFiltered.z;
    }
}

