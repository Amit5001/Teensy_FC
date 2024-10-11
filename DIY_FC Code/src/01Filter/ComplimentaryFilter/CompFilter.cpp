#include <Arduino.h>
#include <01Filter/ComplimentaryFilter/CompFilter.h>
#include <Var_types.h>

#define PI 3.14159265358979323846f
#define rad2deg 180.0f/PI
#define BETA 0.01f
#define USE_MAG 1
quat_t q = {0.0, 0.0, 0.0, 1.0};

static float gravX , gravY, gravZ; // Unit vector in the direction of the estimated gravity

//static float baseZaccel = 1.0; // Base acceleration in the Z direction

void UpdateQ(Measurement_t meas, float dt){
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy, _8qx, _8qy, qwqw, qxqx, qyqy, qzqz;

    // Model based time propagation
    qDot1 = 0.5f * (-q.x * meas.gyro.x - q.y * meas.gyro.y - q.z * meas.gyro.z);
    qDot2 = 0.5f * (q.w * meas.gyro.x + q.y * meas.gyro.z - q.z * meas.gyro.y);
    qDot3 = 0.5f * (q.w * meas.gyro.y - q.x * meas.gyro.z + q.z * meas.gyro.x);
    qDot4 = 0.5f * (q.w * meas.gyro.z + q.x * meas.gyro.y - q.y * meas.gyro.x);

    if(!(meas.acc.x == 0.0 && meas.acc.y ==0.0 && meas.acc.z ==0.0)) {
        // Normalise accelerometer measurement
    recipNorm = invSqrt(meas.acc.x * meas.acc.x + meas.acc.y * meas.acc.y + meas.acc.z * meas.acc.z);
    meas.acc.x *= recipNorm;
    meas.acc.y *= recipNorm;
    meas.acc.z *= recipNorm;

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
    s0 = _4qw * qyqy + _2qy * meas.acc.x + _4qw * qxqx - _2qx * meas.acc.y;
    s1 = _4qx * qzqz - _2qz * meas.acc.x + 4.0f * qwqw * q.x - _2qw * meas.acc.y - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * meas.acc.z;
    s2 = 4.0f * qwqw * q.y + _2qw * meas.acc.x + _4qy * qzqz - _2qz * meas.acc.y - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * meas.acc.z;
    s3 = 4.0f * qxqx * q.z - _2qx * meas.acc.x + 4.0f * qyqy * q.z - _2qy * meas.acc.y;
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
    if(USE_MAG &&( meas.mag.x != 0.0 && meas.mag.y != 0.0 && meas.mag.z != 0.0)){
        float MagNorm = invSqrt(meas.mag.x * meas.mag.x + meas.mag.y * meas.mag.y + meas.mag.z * meas.mag.z);
        meas.mag.x *= MagNorm;
        meas.mag.y *= MagNorm;
        meas.mag.z *= MagNorm;
        _2qw = 2.0f * q.w;
        _2qx = 2.0f * q.x;
        _2qy = 2.0f * q.y;
        _2qz = 2.0f * q.z;

        float bx = sqrt(meas.mag.x * meas.mag.x + meas.mag.y * meas.mag.y);
        float bz = meas.mag.z;
        float _2qwqy = _2qw * q.y;
        float _2qxqz = _2qx * q.z;

        float s0 = _2qxqz - _2qwqy - meas.mag.x;
        float s1 = _2qw * bz - _2qz * bx - meas.mag.y;
        float s2 = _2qw * bx + _2qx * bz - meas.mag.z;
        float s3 = _2qw * bx + _2qz * bx - bz;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // Normalize step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step (Magnetometer)
        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;
    }


    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * dt;
    q.x += qDot2 * dt;
    q.y += qDot3 * dt;
    q.z += qDot4 * dt;

    estimatedGravityDir(&gravX, &gravY, &gravZ);

}



// Returning the estimated quaternion
void GetQuaternion(quat_t* q_){
    q_->x = q.x;
    q_->y = q.y;
    q_->z = q.z;
    q_->w = q.w;

}

void GetEulerRPY(vec3_t* rpy){
    float gx = gravX;
    float gy = gravY;
    float gz = gravZ;

    if (gx > 1) gx = 1;
    if (gx < -1) gx = -1;

    // Currently returend in radians, can be converted to degrees by multiplying by rad2deg
    rpy->x = atan2f(2*(q.w*q.z + q.x*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    rpy->y = asinf(gx);
    rpy->z = atan2f(gy, gz);
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

static float GetAccZ(float ax, float ay, float az) {
    return (ax*gravX + ay*gravY + az*gravZ);
}

static void estimatedGravityDir(float* gx, float* gy, float*gz){
    *gx = 2*(q.x*q.z - q.y*q.w);
    *gy = 2*(q.y*q.z + q.x*q.w);
    *gz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}

