#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <Arduino.h>
#include <Var_types.h>

class Motors {
    private:
        int PWM_MIN = 1000;
        int PWM_MAX = 2000;
        int MOTOR_OFF = 0;
        int MOTOR_ARM = 1100;
        int ESC_FREQUENCY;
        //int PWM_MAX = 4095;
        // int PWM_MAX = 65535;

        motor_t Motor_struct;

    public:
        Motors(int freq, int motor1_pin, int motor2_pin, int motor3_pin, int motor4_pin){
            ESC_FREQUENCY = freq;

            Motor_struct.M1_pin = motor1_pin;
            Motor_struct.M2_pin = motor2_pin;
            Motor_struct.M3_pin = motor3_pin;
            Motor_struct.M4_pin = motor4_pin;
            Motor_struct.M1_PWM = MOTOR_OFF;
            Motor_struct.M2_PWM = MOTOR_OFF;
            Motor_struct.M3_PWM = MOTOR_OFF;
            Motor_struct.M4_PWM = MOTOR_OFF;


        };
        void Motors_init();
        void Arm();
        void Disarm();
        void set_motorPWM();
        void Motor_Mix(attitude_t motor_input, int throttle);
        motor_t Get_motor();

};

#endif // _MOTORS_CONTROL_H_