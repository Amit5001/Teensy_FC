/*
This code implementing the Kalman Filter on 2 IMUs, estimating roll and pitch.
Might want to add the yaw estimation as well and a low pass filter for the gyro data, high pass filter for the accelerometer data.

*/

#ifndef IMUCOMBINER_H
#define IMUCOMBINER_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
#include "KalmanFilter.h"
#include <vector_type.h>

#define IMU_timer 1000 // IMU timer in Hz
#define GYRO_NOISE 7*sqrt(IMU_timer)/1000  // 7 is the noise density of the gyro in mdps/sqrt(Hz)
#define ACC_NOISE 90*sqrt(IMU_timer)/1000000  // 90 is the noise density of the accelerometer in ug/sqrt(Hz)


// 0-2 acc, 3-5 gyro
template<int Nstate=4, int Nmeas =4>
class IMUCombiner{
  private:
    KalmanFilter<Nstate, Nmeas> KF;

    // Kalman FIlter Params:
    float g =9.81;
    float dt = 1/IMU_timer;

  public:
    float est[4]; // Initial estimate
    IMUCombiner( int Pgain=1, int Qgain=1, int Rgain=1){
      // Set up the Kalman Filter Pololu:
      KF.epsilon_max = 1; // Maximum error
      KF.Q_scale_factor = 2.0; // Scaling factor for Q
      KF.count = 0; // Counter for scaling Q
      KF.x.Fill(0.0); // State vector initialization x= {roll, pitch, roll rate, pitch rate}
      KF.y.Fill(0.0); // Measurement vector initialization
      KF.z.Fill(0.0); // Error vector initialization
      KF.u.Fill(0.0); // Input control vector initialization    
      KF.A = {1.0, 0.0, dt, 0.0, 
                0.0, 1.0, 0.0, dt,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0}; // State transition matrix
      KF.B.Fill(0.0); // Control matrix
      KF.C = {1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0}; // Observation matrix
      KF.P = {1, 0.0, 0.0, 0.0, 
                0.0, 1, 0.0, 0.0,
                0.0, 0.0, 1, 0.0,
                0.0, 0.0, 0.0, 1}; // State covariance matrix
      KF.R = {ACC_NOISE, 0.0, 0.0, 0.0, 
                  0.0, ACC_NOISE, 0.0, 0.0,
                  0.0, 0.0, ACC_NOISE, 0.0,
                  0.0, 0.0, 0.0, ACC_NOISE}; // Measurement covariance matrix
      KF.Q = {GYRO_NOISE, 0.0f, 0.0f, 0.0f,
                  0.0f, GYRO_NOISE, 0.0f, 0.0f,
                  0.0f, 0.0f, GYRO_NOISE, 0.0f,
                  0.0f, 0.0f, 0.0f, GYRO_NOISE}; // Process noise covariance matrix

      for (int i = 0; i < Nstate; ++i) {
          for (int j = 0; j < Nstate; ++j) {
              KF.P(i, j) *= Pgain; 
              KF.Q(i, j) *= Qgain;
              KF.R(i, j) *= Rgain; 
          }
    }
    KF.K.Fill(0.0); // Kalman gain initialization
    }
    

  float* combine_imu(float IMU_Pol[6], float MPU6050[6]){
    float accX = 0.35f * MPU6050[0] + 0.65f * IMU_Pol[0];
    float accY = 0.35f * MPU6050[1] + 0.65f * IMU_Pol[1];
    float accZ = 0.35f * MPU6050[2] + 0.65f * IMU_Pol[2];
    float gyroX = 0.35f * MPU6050[3] + 0.65f * IMU_Pol[3];
    float gyroY = 0.35f * MPU6050[4] + 0.65f * IMU_Pol[4];
    //float gyroZ = 0.35f * MPU6050[5] + 0.65f * IMU_Pol[5];
  
    KF.z(0) = atan2(accY,accZ) ; // Roll in radians
    KF.z(1) = atan2(accX,sqrt(pow(accY,2) + pow(accZ,2))); // Pitch in radians
    KF.z(2) = gyroX; // Roll rate in radians per second
    KF.z(3) = gyroY; // Pitch rate in radians per second
    KF.get_prediction();
    KF.get_kalman_gain();
    KF.get_update();
    KF.get_residual();
    KF.get_epsilon();
    KF.scale_Q();

    est[0] = KF.x(0);
    est[1] = KF.x(1);
    est[2] = KF.x(2);
    est[3] = KF.x(3);

    return est;

  }

  float* one_imu( vec3_t acc, vec3_t gyro){

  
    KF.z(0) = atan2(acc.y,acc.z) ; // Roll in radians
    KF.z(1) = atan2(acc.x,sqrt(pow(acc.y,2) + pow(acc.z,2))); // Pitch in radians
    KF.z(2) = gyro.x; // Roll rate in radians per second
    KF.z(3) = gyro.y; // Pitch rate in radians per second
    KF.get_prediction();
    KF.get_kalman_gain();
    KF.get_update();
    KF.get_residual();
    KF.get_epsilon();
    KF.scale_Q();

    est[0] = KF.x(0);
    est[1] = KF.x(1);
    est[2] = KF.x(2);
    est[3] = KF.x(3);

    return est;

  }



};

#endif