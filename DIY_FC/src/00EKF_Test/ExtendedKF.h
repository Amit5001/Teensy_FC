#ifndef ATTITUDE_KALMAN_FILTER_H
#define ATTITUDE_KALMAN_FILTER_H

#include "eigen/Dense"
#include <cmath>

class AttitudeKalmanFilter {
public:
    // Constructor
    AttitudeKalmanFilter();
    
    // Initialize the filter
    void initialize(const Eigen::Vector3d& initial_accel);
    
    // Main filter update step
    void update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt);
    
    // Getters
    Eigen::Vector4d getQuaternion() const { return q_; }
    Eigen::Vector3d getAngularVelocity() const { return omega_; }
    Eigen::Vector3d getGyroBias() const { return bias_; }
    Eigen::Matrix3d getRotationMatrix() const;
    Eigen::Vector3d getEulerAngles() const; // Roll, Pitch, Yaw
    
    // Setters for tuning
    void setProcessNoise(double gyro_noise, double bias_noise);
    void setMeasurementNoise(double accel_noise);
    
private:
    // State vector: [quaternion(4), angular_velocity(3), gyro_bias(3)] = 10 elements
    // But we use 9x9 error state covariance (quaternion error is 3D)
    static constexpr int STATE_SIZE = 9;
    static constexpr int MEASUREMENT_SIZE = 3;
    
    // State variables
    Eigen::Vector4d q_;      // Quaternion [w, x, y, z]
    Eigen::Vector3d omega_;  // Angular velocity (rad/s)
    Eigen::Vector3d bias_;   // Gyro bias (rad/s)
    
    // Covariance matrix (9x9 for error state)
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P_;
    
    // Noise matrices
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q_;  // Process noise
    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R_;  // Measurement noise
    
    // IMU parameters for LSM6DSO
    static constexpr double GYRO_SCALE = 250.0 * M_PI / (180.0 * 32768.0);  // 250 dps full scale
    static constexpr double ACCEL_SCALE = 2.0 * 9.81 / 32768.0;              // 2g full scale
    static constexpr double GRAVITY = 9.81;
    
    // Internal methods
    void predict(const Eigen::Vector3d& gyro, double dt);
    void correct(const Eigen::Vector3d& accel);
    
    // Quaternion operations
    Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);
    Eigen::Vector4d quaternionFromAxisAngle(const Eigen::Vector3d& axis_angle);
    void normalizeQuaternion();
    Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& q);
    Eigen::Matrix<double, 4, 3> quaternionErrorMatrix(const Eigen::Vector4d& q);
    
    // Skew symmetric matrix
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
};

// Implementation


#endif // ATTITUDE_KALMAN_FILTER_H