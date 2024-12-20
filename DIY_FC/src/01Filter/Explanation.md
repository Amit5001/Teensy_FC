# Complimentary Filter for Attitude Estimation

This filter is first layer of filtering. using Gyro, Accelerometer and Magnetometer measurements in order to estimate the attitude of the drone.
The filter is based on:

1. https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/estimator/estimator_complementary.c
2. https://github.com/bitcraze/crazyflie-firmware/blob/3a35c64953d5831194e3a283d6302ffc6e318978/src/modules/src/sensfusion6.c#L303
3. https://github.com/arduino-libraries/MadgwickAHRS/blob/master/src/MadgwickAHRS.cpp

The filter changes dynamically the beta value, depends on the magnitude of the gyro measurements.