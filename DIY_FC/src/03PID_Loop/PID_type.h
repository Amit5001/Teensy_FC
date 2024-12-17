#ifndef PID_TYPE_H
#define PID_TYPE_H

#include <Arduino.h>
#include <Var_types.h>

void initializePIDParams(float RrollPID[3] = nullptr, float RyawPID[3] = nullptr,
                         float Imax_rate[2] = nullptr, float SrollPID[3] = nullptr,
                         float SyawPID[3] = nullptr, float Imax_stab[2] = nullptr);
attitude_t PID_rate(attitude_t des_rate, attitude_t rate, float DT);
attitude_t PID_stab(attitude_t des_angle, attitude_t angle, float DT);


#endif