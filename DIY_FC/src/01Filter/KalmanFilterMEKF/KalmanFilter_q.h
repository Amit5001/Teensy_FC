/*
This will be a try to implement Multiplicative Extended Kalman Filter for attitude estimation using quaternions.
We will use the Basic Linear Algebra library for the matrix operations while generating new matrices for the skew symmetric matrix and the quaternion multiplication.
The filter will estimate the rotation quaternion and the gyro bias.
The state vector will be of size 7, with the first 4 elements being the quaternion and the next 3 being the gyro bias.
The measurement vector will be the accelerometer data.

The quaternion is described as follows: q={e1, e2, e3, q} where q is the scalar part of the quaternion and e1, e2, e3 are the vector part of the quaternion.
*/

// NEED TO CONTUNUE

#ifndef KalmanFilter_q_h
#define KalmanFilter_q_h

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <vector_type.h>
using namespace BLA;
#define IMU_timer 1000 // IMU timer in Hz
#define WALK_SIGMA 2.036*pow(10,-5)  // rad/sqrt(s) - Walk standard deviation




template<int Nstate=7, int Nmeas=3>
class KalmanFilter{
    private:
        BLA::Matrix<Nstate, Nstate> I = createDiagonalMatrix(Matrix<3> v={1,1,1}); // Identity matrix

    public:
    KalmanFilter(int Pgain, int Qgain, int Rgain){
        float dt;
        BLA::Matrix<Nstate, 1> x.fill(0.0); // State vector first 4 elements are quaternion, next 3 are gyro bias initializing to 0
        BLA::Matrix<4> delta_q = {1, 0, 0, 0}; // Initial eror quaternion
        BLA::Matrix<3> bias_dot.fill(GYRO_NOISE); // Initial gyro bias rate

    };

    void updateTimer() {
        uint32_t time_now = micros();
        uint32_t change_time = time_now - last_time;
        last_time = time_now;
        dt = float(change_time)*1e-6;         
    }  

    // MUST MAKE SURE THAT THE QUATERNION IS ALWAYS IN THE SAME ORDER
    Matrix<4> update_quat(Matrix<3> gyro, Matrix<3> acc){
        updateTimer();
        // Predict step
        Matrix<3> bias = {x(4), x(5), x(6)}; // Gyro bias estimation
        Matrix<4> q = {x(0), x(1), x(2), x(3)}; // Quaternion estimation
        Matrix<3> w = gyro - bias; //measured gyro minus estimated bias
        Matrix<4> q_dot = 0.5*quat_mult(q,{w(0), w(1), w(2), 0});
        q = q + q_dot*dt;
        q = q/q.Norm();

        // Update step
        Matrix<3> acc_meas = quat_mult(quat_mult(q, {0, acc(0), acc(1), acc(2)}), quat_conj(q));
        Matrix<3> e = acc_meas - {0, 0, 1};
        Matrix<3> e_cross = createSkewSymmetric(e);
        Matrix<3> q_conj = {q(0), -q(1), -q(2), -q(3)};
        Matrix<3> q_dot = 0.5*quat_mult({0, e_cross(0), e_cross(1), e_cross(2)}, q_conj);
        q = q + q_dot*dt;
        q = q/q.Norm();

        return q;
    }



  

    
    // Function for quaternion multiplication
    Matrix<4> quat_mult(const Matrix<4>& q1, const Matrix<4>& q2){
        Matrix<4> q;
        BLA::Matrix <3> e1 = {q1(0), q1(1), q1(2)};
        BLA::Matrix <3> e2 = {q2(0), q2(1), q2(2)};
        BLA::Matrix<3,3>cross_prod = CrossProduct(e1, e2);
        BLA::Matrix<3> dot_prod = DotProduct(~e1, e2);

        BLA::Matrix<3> e3 = q1(3)*e2 + q2(3)*e1 + cross_prod;
        BLA::Matrix<1> w3 = q1{3}*q2{3} - dot_prod;
        
        q = {e3(0), e3(1), e3(2), w3(0)};
        return q;
    }


    // Function to create diagonal Matrix
    Matrix<3, 3> createDiagonalMatrix(const Matrix<3>& v) {
        Matrix<3, 3> diagonal;

        // Set the diagonal elements based on the input vector v = {v1, v2, v3}
        diagonal.Fill(0); // Initialize all elements to 0

        diagonal(0, 0) = v(0);
        diagonal(1, 1) = v(1);
        diagonal(2, 2) = v(2);

        return diagonal;
    }

    // Function to create a skew-symmetric matrix from a 3D vector
    Matrix<3, 3> createSkewSymmetric(const Matrix<3>& v) {
        Matrix<3, 3> skew;

        // Set the skew-symmetric matrix elements based on the input vector v = {v1, v2, v3}
        skew(0, 0) = 0;       skew(0, 1) = -v(2);   skew(0, 2) = v(1);
        skew(1, 0) = v(2);    skew(1, 1) = 0;       skew(1, 2) = -v(0);
        skew(2, 0) = -v(1);   skew(2, 1) = v(0);    skew(2, 2) = 0;

        return skew;
    }

    // Function to create Big Omega matrix from a 3D vector
    Matrix<3,3> createOMEGA(const Matrix<3>& w){
        Matrix<4,4> OMEGA;
        OMEGA(0,0) = 0;     OMEGA(0,1) = w(2);   OMEGA(0,2) = -w(1);   OMEGA(0,3) = w(0);
        OMEGA(1,0) = -w(2);  OMEGA(1,1) = 0;       OMEGA(1,2) = w(0);    OMEGA(1,3) = w(1);
        OMEGA(2,0) = w(1);  OMEGA(2,1) = -w(0);   OMEGA(2,2) = 0;       OMEGA(2,3) = w(2);
        OMEGA(3,0) = -w(0);  OMEGA(3,1) = -w(1);    OMEGA(3,2) = -w(2);   OMEGA(3,3) = 0;

        return OMEGA;
    }
    // Function to calculate the rotation matrix from the quaternion - used with the best estimate of the quaternion from the filter
    Matrix<3,3> calcR(const Matrix<4>& q){
        // q = {e1, e2, e3, q} = {q0, q1, q2, q3}
        Matrix<3,3> R;
        R(0,0) = 1- 2*(pow(q(1),2) + pow(q(2),2));
        R(0,1) = 2*(q(0)*q(1)-q(2)*q(3));
        R(0,2) = 2*(q(0)*q(2)+q(1)*q(3));
        R(1,0) = 2*(q(0)*q(1)+q(2)*q(3));
        R(1,1) = 1- 2*(pow(q(0),2) + pow(q(2),2));
        R(1,2) = 2*(q(1)*q(2)-q(0)*q(3));
        R(2,0) = 2*(q(0)*q(2)-q(1)*q(3));
        R(2,1) = 2*(q(1)*q(2)+q(0)*q(3));
        R(2,2) = 1- 2*(pow(q(0),2) + pow(q(1),2));

        return R;

    }
};

#endif