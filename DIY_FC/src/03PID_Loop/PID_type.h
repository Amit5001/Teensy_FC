#ifndef PID_TYPE_H
#define PID_TYPE_H

#include <Arduino.h>
#include <Var_types.h>

void initializePIDParams(float RrollPID[3]={0.1,0.01,0.01}, float RyawPID[3]={0.1,0.01,0.01},
                         float Imax_rate[2]={100.0f,100.0f}, float SrollPID[3]={0.1,0.01,0.01},
                         float SyawPID[3]={0.1,0.01,0.01}, float Imax_stab[2]={100.0f,100.0f});
attitude_t PID_rate(attitude_t des_rate, attitude_t rate, float DT);
attitude_t PID_stab(attitude_t des_angle, attitude_t angle, float DT);


#endif