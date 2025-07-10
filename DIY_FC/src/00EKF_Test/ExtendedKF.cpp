#include "ExtendedKF.h"

AttitudeKalmanFilter::AttitudeKalmanFilter() {
    // Initialize state
    q_ = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
    omega_ = Eigen::Vector3d::Zero();
    bias_ = Eigen::Vector3d::Zero();
    
    // Initialize covariance matrix
    P_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity() * 0.1;
    
    // Default noise parameters
    setProcessNoise(0.01, 0.001);  // Gyro noise, bias noise
    setMeasurementNoise(0.1);      // Accelerometer noise
}

void AttitudeKalmanFilter::initialize(const Eigen::Vector3d& initial_accel) {
    // Initialize attitude from accelerometer (assuming stationary)
    Eigen::Vector3d accel_normalized = initial_accel.normalized();
    Eigen::Vector3d gravity_vec(0, 0, -1);  // Gravity in NED frame
    
    // Calculate initial roll and pitch from accelerometer
    double roll = atan2(accel_normalized.y(), accel_normalized.z());
    double pitch = atan2(-accel_normalized.x(), 
                        sqrt(accel_normalized.y() * accel_normalized.y() + 
                             accel_normalized.z() * accel_normalized.z()));
    
    // Convert to quaternion (yaw = 0)
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    
    q_(0) = cr * cp;  // w
    q_(1) = sr * cp;  // x
    q_(2) = cr * sp;  // y
    q_(3) = 0.0;      // z (yaw = 0)
    
    normalizeQuaternion();
}

void AttitudeKalmanFilter::update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) {
    predict(gyro, dt);
    correct(accel);
}

void AttitudeKalmanFilter::predict(const Eigen::Vector3d& gyro, double dt) {
    // Corrected angular velocity (remove bias)
    Eigen::Vector3d omega_corrected = gyro - bias_;
    omega_ = omega_corrected;
    
    // Propagate quaternion
    Eigen::Vector3d delta_angle = omega_corrected * dt;
    Eigen::Vector4d dq = quaternionFromAxisAngle(delta_angle);
    q_ = quaternionMultiply(q_, dq);
    normalizeQuaternion();
    
    // Build state transition matrix F (9x9)
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F = 
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
    
    // Quaternion error dynamics
    F.block<3, 3>(0, 3) = -0.5 * dt * Eigen::Matrix3d::Identity();
    F.block<3, 3>(0, 6) = 0.5 * dt * Eigen::Matrix3d::Identity();
    
    // Propagate covariance
    P_ = F * P_ * F.transpose() + Q_ * dt;
}

void AttitudeKalmanFilter::correct(const Eigen::Vector3d& accel) {
    // Expected gravity vector in body frame
    Eigen::Matrix3d R_body_to_world = quaternionToRotationMatrix(q_);
    Eigen::Vector3d gravity_world(0, 0, GRAVITY);
    Eigen::Vector3d expected_accel = R_body_to_world.transpose() * gravity_world;
    
    // Innovation (measurement residual)
    Eigen::Vector3d innovation = accel - expected_accel;
    
    // Measurement matrix H (3x9)
    Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> H = 
        Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE>::Zero();
    
    // H for quaternion error (first 3 states)
    H.block<3, 3>(0, 0) = 2.0 * skewSymmetric(expected_accel);
    
    // Innovation covariance
    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S = 
        H * P_ * H.transpose() + R_;
    
    // Kalman gain
    Eigen::Matrix<double, STATE_SIZE, MEASUREMENT_SIZE> K = P_ * H.transpose() * S.inverse();
    
    // State update (error state)
    Eigen::Matrix<double, STATE_SIZE, 1> delta_x = K * innovation;
    
    // Apply error state to nominal state
    // Quaternion update
    Eigen::Vector3d delta_angle = delta_x.head<3>();
    Eigen::Vector4d dq = quaternionFromAxisAngle(delta_angle);
    q_ = quaternionMultiply(dq, q_);
    normalizeQuaternion();
    
    // Angular velocity update
    omega_ += delta_x.segment<3>(3);
    
    // Bias update
    bias_ += delta_x.tail<3>();
    
    // Covariance update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I = 
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
    P_ = (I - K * H) * P_;
}

Eigen::Vector4d AttitudeKalmanFilter::quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    Eigen::Vector4d result;
    result(0) = q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
    result(1) = q1(0) * q2(1) + q1(1) * q2(0) + q1(2) * q2(3) - q1(3) * q2(2);
    result(2) = q1(0) * q2(2) - q1(1) * q2(3) + q1(2) * q2(0) + q1(3) * q2(1);
    result(3) = q1(0) * q2(3) + q1(1) * q2(2) - q1(2) * q2(1) + q1(3) * q2(0);
    return result;
}

Eigen::Vector4d AttitudeKalmanFilter::quaternionFromAxisAngle(const Eigen::Vector3d& axis_angle) {
    double angle = axis_angle.norm();
    if (angle < 1e-8) {
        return Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    }
    
    Eigen::Vector3d axis = axis_angle / angle;
    double half_angle = angle * 0.5;
    double sin_half = sin(half_angle);
    
    Eigen::Vector4d q;
    q(0) = cos(half_angle);
    q(1) = axis(0) * sin_half;
    q(2) = axis(1) * sin_half;
    q(3) = axis(2) * sin_half;
    
    return q;
}

void AttitudeKalmanFilter::normalizeQuaternion() {
    double norm = q_.norm();
    if (norm > 1e-8) {
        q_ /= norm;
    }
}

Eigen::Matrix3d AttitudeKalmanFilter::quaternionToRotationMatrix(const Eigen::Vector4d& q) {
    double w = q(0), x = q(1), y = q(2), z = q(3);
    
    Eigen::Matrix3d R;
    R << 1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
         2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
         2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y);
    
    return R;
}

Eigen::Matrix3d AttitudeKalmanFilter::getRotationMatrix() const {
    return quaternionToRotationMatrix(q_);
}

Eigen::Vector3d AttitudeKalmanFilter::getEulerAngles() const {
    Eigen::Matrix3d R = getRotationMatrix();
    
    // Extract Euler angles (roll, pitch, yaw) from rotation matrix
    double roll = atan2(R(2,1), R(2,2));
    double pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
    double yaw = atan2(R(1,0), R(0,0));
    
    return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Matrix3d AttitudeKalmanFilter::skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew;
    skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return skew;
}

void AttitudeKalmanFilter::setProcessNoise(double gyro_noise, double bias_noise) {
    Q_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Zero();
    
    // Quaternion error noise
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise;
    
    // Angular velocity noise
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise;
    
    // Bias noise
    Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * bias_noise * bias_noise;
}

void AttitudeKalmanFilter::setMeasurementNoise(double accel_noise) {
    R_ = Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE>::Identity() * 
         accel_noise * accel_noise;
}