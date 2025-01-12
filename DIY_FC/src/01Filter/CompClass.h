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


class NotchFilter {
public:
    NotchFilter(float sample_freq, float notch_freq, float bandwidth);
    float update(float input);    
private:
    notch_filter_t* filter_data;
};

class CompFilter {
public:
    CompFilter(bool _MAG = true, float Notch_freq = 600.0f, float acc_notch_bw = 30.0f, float gyro_notch_bw = 45.0f) : USE_MAG(_MAG) {
        q = {0.0f, 0.0f, 0.0f, 1.0f};
            // Initialize notch filters
        float sample_freq = RATE_1000_HZ;  // Assuming 1kHz sample rate

        // Store filter parameters
        this->Notch_freq = Notch_freq;
        this->acc_notch_bw = acc_notch_bw;
        this->gyro_notch_bw = gyro_notch_bw;

        acc_notch_filters[0] = new NotchFilter(sample_freq, Notch_freq, acc_notch_bw);
        acc_notch_filters[1] = new NotchFilter(sample_freq, Notch_freq, acc_notch_bw);
        acc_notch_filters[2] = new NotchFilter(sample_freq, Notch_freq, acc_notch_bw);

        // Initialize notch filters for gyro - wider bandwidth and slightly different frequency
        // because gyros might pick up harmonics differently
        gyro_notch_filters[0] = new NotchFilter(sample_freq, Notch_freq, gyro_notch_bw);
        gyro_notch_filters[1] = new NotchFilter(sample_freq, Notch_freq, gyro_notch_bw);
        gyro_notch_filters[2] = new NotchFilter(sample_freq, Notch_freq, gyro_notch_bw);
    }

    // Core functions
    void UpdateQ(Measurement_t* meas, float dt);
    void GetQuaternion(quat_t* q_);
    void GetEulerRPYrad(attitude_s* rpy, float initial_heading);
    void GetEulerRPYdeg(attitude_s* rpy, float initial_heading);
    
private:
    // Core variables
    bool USE_MAG;
    quat_t q;
    float gyroNorm;
    float gravX, gravY, gravZ;

    // Notch filter parameters
    float Notch_freq;
    float acc_notch_bw;
    float gyro_notch_bw;

    // Utility functions
    void InitialFiltering(Measurement_t* meas);
    float calculateDynamicBeta(Measurement_t meas);
    float invSqrt(float x);
    void estimatedGravityDir(float* gx, float* gy, float* gz);
    float GetAccZ(float ax, float ay, float az);
    
    // Notch filter instances
    NotchFilter* acc_notch_filters[3];
    NotchFilter* gyro_notch_filters[3];
};


#endif