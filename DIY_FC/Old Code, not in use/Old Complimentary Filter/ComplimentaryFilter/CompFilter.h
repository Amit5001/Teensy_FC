#ifndef COMPFILTER_H
#define COMPFILTER_H

#include <Arduino.h>
#include <Var_types.h>
void UpdateQ(Measurement_t*, float);
void InitialFiltering(Measurement_t*);

void GetQuaternion(quat_t* );
void GetEulerRPYrad(vec3_t* );
void GetEulerRPYdeg(vec3_t* ,float);

//---------------------------------------------------------------------------------------------------
float invSqrt(float);
float GetAccZ(float, float, float);
void estimatedGravityDir(float*, float*, float*);
float calculateDynamicBeta(Measurement_t);




#endif