#ifndef KalmanFilter_h
#define KalmanFilter_h

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;



template<int Nstate=4, int Nmeas=4>
class KalmanFilter{
    private:
        BLA::Matrix<Nstate, Nstate> I; // Identity matrix

    public:
        KalmanFilter() {
        // Manually setting identity matrix
            for (int i = 0; i < Nstate; i++) {
                for (int j = 0; j < Nstate; j++) {
                    I(i, j) = (i == j) ? 1.0 : 0.0;
                }
            }
        }
        BLA::Matrix<Nstate, 1> x; // State vector
        BLA::Matrix<Nstate, Nstate> P; // State covariance matrix
        BLA::Matrix<Nmeas, 1> y; // Measurement vector
        BLA::Matrix<Nmeas, 1> z; // error vector - meas - state prediction
        BLA::Matrix<Nmeas, 1> u; // Input control vector
        BLA::Matrix<Nmeas, Nmeas> R; // Measurement covariance matrix
        BLA::Matrix<Nstate, Nmeas> C; // Measurement matrix
        BLA::Matrix<Nstate, Nstate> Q; // Process noise covariance matrix
        BLA::Matrix<Nstate, Nstate> A; // State transition matrix
        BLA::Matrix<Nstate, Nstate> B; // Control matrix
        BLA::Matrix<Nstate, Nmeas> K; // Kalman gain
        BLA::Matrix<Nmeas, Nmeas> inv; // Inverse of the measurement covariance matrix
        BLA::Matrix<Nmeas, 1> e; // error - meas - state prediction
        BLA::Matrix<1, 1> epsilon = {0.0}; // Error
        float epsilon_max = 1; // Maximum error
        float Q_scale_factor = 2.0; // Scaling factor for Q
        int count = 0; // Counter for scaling Q

        void get_prediction();
        void get_kalman_gain();
        void get_update();
        void get_residual();
        void get_epsilon();
        void scale_Q();
};

template<int Nstate, int Nmeas>
void KalmanFilter<Nstate, Nmeas>::get_prediction(){
    this->x = (this->A*this->x) + (this->B*this->u); // Based on the model.
    this->P = (this->A * (this->P * ~(this->A))) + this->Q;
};


template<int Nstate, int Nmeas>
void KalmanFilter<Nstate, Nmeas>::get_kalman_gain(){
    this->inv = BLA::Inverse((this->C * (this->P * (~this->C))) + this->R);
    this->K = ((this->P * (~this->C)) * this->inv);
};

template<int Nstate, int Nmeas>
void KalmanFilter<Nstate, Nmeas>::get_update(){
    this->x = this->x + (this->K * (this->y - (this->C * this->x)));
    this->P = (((this->I - (this->K * this->C)) * this->P * (~(this->I - (this->K * this->C)))) + ((this->K * this->R) * (~this->K)));
};

template<int Nstate, int Nmeas>
void KalmanFilter<Nstate, Nmeas>::get_residual(){
    this->y = this->z - this->x;
};

template<int Nstate, int Nmeas>
void KalmanFilter<Nstate, Nmeas>::get_epsilon(){
    this->epsilon = (~this->y * (this->inv * this->y));
};

template<int Nstate, int Nmeas>
void KalmanFilter<Nstate, Nmeas>::scale_Q(){
    if (abs(this->epsilon(0)) > this->epsilon_max){
        this->Q *= this->Q_scale_factor;
        this->count += 1;
    }
};

#endif