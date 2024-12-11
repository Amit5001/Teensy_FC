#include <Arduino.h>
#include <Var_types.h>
#include <MotorsControl.h>

// Initialization of the motors
void Motors::Motors_init(){
    // Set the PWM frequency for the ESCs
    analogWriteFrequency(Motor_struct.M1_pin, ESC_FREQUENCY);
    analogWriteFrequency(Motor_struct.M2_pin, ESC_FREQUENCY);
    analogWriteFrequency(Motor_struct.M3_pin, ESC_FREQUENCY);
    analogWriteFrequency(Motor_struct.M4_pin, ESC_FREQUENCY);

    // Set the PWM resolution for the ESCs
    analogWriteResolution(12);

    // Set the PWM value for the ESCs
    analogWrite(Motor_struct.M1_pin, PWM_MIN);
    analogWrite(Motor_struct.M2_pin, PWM_MIN);
    analogWrite(Motor_struct.M3_pin, PWM_MIN);
    analogWrite(Motor_struct.M4_pin, PWM_MIN);
}

// Arm Function
void Motors::Arm(){
    // Set the PWM value for the ESCs
    analogWrite(Motor_struct.M1_pin, MOTOR_ARM);
    analogWrite(Motor_struct.M2_pin, MOTOR_ARM);
    analogWrite(Motor_struct.M3_pin, MOTOR_ARM);
    analogWrite(Motor_struct.M4_pin, MOTOR_ARM);
}


// Disarm Function
void Motors::Disarm(){
    // Set the PWM value for the ESCs
    analogWrite(Motor_struct.M1_pin, MOTOR_OFF);
    analogWrite(Motor_struct.M2_pin, MOTOR_OFF);
    analogWrite(Motor_struct.M3_pin, MOTOR_OFF);
    analogWrite(Motor_struct.M4_pin, MOTOR_OFF);
}

void Motors::set_motorPWM(){
    // Set the PWM value for the ESCs
    analogWrite(Motor_struct.M1_pin, Motor_struct.PWM1);
    analogWrite(Motor_struct.M2_pin, Motor_struct.PWM2);
    analogWrite(Motor_struct.M3_pin, Motor_struct.PWM3);
    analogWrite(Motor_struct.M4_pin, Motor_struct.PWM4);
}

void Motors::Motor_Mix(attitude_t motor_input, int throttle) {

    Motor_struct.PWM1 = throttle + motor_input.roll + motor_input.pitch + motor_input.yaw;
    Motor_struct.PWM2 = throttle + motor_input.roll - motor_input.pitch - motor_input.yaw;
    Motor_struct.PWM3 = throttle - motor_input.roll - motor_input.pitch + motor_input.yaw;
    Motor_struct.PWM4 = throttle - motor_input.roll + motor_input.pitch - motor_input.yaw;

    // Constrain the values to be between 1100 and 1900
    Motor_struct.PWM1 = constrain(Motor_struct.PWM1, 1100, 1900);
    Motor_struct.PWM2 = constrain(Motor_struct.PWM2, 1100, 1900);
    Motor_struct.PWM3 = constrain(Motor_struct.PWM3, 1100, 1900);
    Motor_struct.PWM4 = constrain(Motor_struct.PWM4, 1100, 1900);
}


motor_t Motors::Get_motor(){
    return Motor_struct;
}