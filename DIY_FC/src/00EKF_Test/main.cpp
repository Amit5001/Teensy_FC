#include <Arduino.h>
#include "Eigen.h"
#include "ExtendedKF.h"


void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB
    }
    
    // Initialize the Kalman filter
    AttitudeKalmanFilter ekf;
    Eigen::Vector3d initial_accel(0.0, 0.0, -9.81); // Assuming stationary at start
    ekf.initialize(initial_accel);
    
    // Example gyro and accel data
    Eigen::Vector3d gyro(0.01, 0.02, 0.03); // Example gyro data in rad/s
    Eigen::Vector3d accel(0.0, 0.0, -9.81); // Example accel data in m/s^2
    double dt = 0.01; // Time step in seconds
    
    // Update the filter with new measurements
    ekf.update(gyro, accel, dt);
    
    // Print the estimated quaternion and Euler angles
    auto quat = ekf.getQuaternion();
    Serial.print("Estimated Quaternion: ");
    Serial.print("w: "); Serial.print(quat(0)); Serial.print(" ");
    Serial.print("x: "); Serial.print(quat(1)); Serial.print(" ");
    Serial.print("y: "); Serial.print(quat(2)); Serial.print(" ");
    Serial.print("z: "); Serial.println(quat(3));
    
    auto euler = ekf.getEulerAngles();
    Serial.print("Estimated Euler Angles (Roll, Pitch, Yaw): ");
    Serial.print("Roll: "); Serial.print(euler(0)); Serial.print(" ");
    Serial.print("Pitch: "); Serial.print(euler(1)); Serial.print(" ");
    Serial.print("Yaw: "); Serial.println(euler(2));
}

void loop() {
    // Nothing to do here
    delay(1000); // Just to avoid flooding the serial output
}