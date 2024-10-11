#ifndef COMPFILTER_H
#define COMPFILTER_H

#include <Arduino.h>
#include <Var_types.h>
void UpdateQ(Measurement_t, float);


void GetQuaternion(quat_t* );
void GetEulerRPY(vec3_t* );

//---------------------------------------------------------------------------------------------------
float invSqrt(float );
static float GetAccZ(float, float, float);
static void estimatedGravityDir(float*, float*, float*);



#endif