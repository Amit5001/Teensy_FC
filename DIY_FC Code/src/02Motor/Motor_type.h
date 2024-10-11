#ifndef MOTOR_TYPE_H
#define MOTOR_TYPE_H


#include <Arduino.h>

typedef struct Motor_s{
    int pin;
    int PWM_val;

}Motor_t;

class Motors {
    private:
        int MOTOR_MIN = 1000;
        int MOTOR_MAX = 2000;
        int MOTOR_OFF = 0;
        int MOTOR_ARM = 1100;
        int ESC_FREQUENCY;
        Motor_t motor1;
        Motor_t motor2;
        Motor_t motor3;
        Motor_t motor4;

    public:
        Motors(int freq, int motor1_pin, int motor2_pin, int motor3_pin, int motor4_pin){
            ESC_FREQUENCY = freq;
            motor1 = {motor1_pin, MOTOR_OFF};
            motor2 = {motor2_pin, MOTOR_OFF};
            motor3 = {motor3_pin, MOTOR_OFF};
            motor4 = {motor4_pin, MOTOR_OFF};
        };
        void motor_calibration();
        void setup_pins();
        void set_motor(Motor_t, int);
        void Initialization();
        void Arm();
        void Disarm();

        // Getters and Setters
        int get_motor_pin(Motor_t motor);
        int get_motor_PWM(Motor_t motor);
        void set_motor_pin(Motor_t motor, int pin);
        void set_motor_PWM(Motor_t motor, int PWM);

};


#endif