#ifndef PID_TYPE_H
#define PID_TYPE_H

#include <Arduino.h>
#include <Var_types.h>

float PID_rate(float des_rate, float rate); // PID controller for rate
float PID_stab(float des_angle, float angle, float rate); // PID controller for stabilization

#endif