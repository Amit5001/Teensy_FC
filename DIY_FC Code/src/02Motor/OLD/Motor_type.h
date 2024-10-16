#ifndef MOTOR_TYPE_H
#define MOTOR_TYPE_H


#include <Arduino.h>
#include <Servo.h>

#define PWM_RES15 15
#define PWM_RES16 16
#define PWM_RES12 12
#define PWM_RES8 8

typedef struct Motor_s{
    Servo motorS;
    int pin;
    int PWM_val;

}Motor_t;

class Motors {
    private:
        // int MOTOR_MIN = 1060;
        // int MOTOR_MAX = 1860;
        int MOTOR_MIN = 1000;
        int MOTOR_MAX = 2000;
        int MOTOR_OFF = 0;
        int MOTOR_ARM = 1100;
        int ESC_FREQUENCY;
        if (PWM_RES15){
            int PWM_RESOLUTION = 15; // Max PWM resolution of the teensy4.1
            int PWM_MAX = 32767;
        } else if (PWM_RES16){
            int PWM_RESOLUTION = 16; // Max PWM resolution of the teensy4.1
            int PWM_MAX = 65535;
        } else if (PWM_RES12){
            int PWM_RESOLUTION = 12; // Max PWM resolution of the teensy4.1
            int PWM_MAX = 4095;
        } else if (PWM_RES8){
            int PWM_RESOLUTION = 18; // Max PWM resolution of the teensy4.1
            int PWM_MAX = 255;
        }

        Motor_t _motor1;
        Motor_t _motor2;
        Motor_t _motor3;
        Motor_t _motor4;

    public:
        Motors(int freq, int motor1_pin, int motor2_pin, int motor3_pin, int motor4_pin){
            ESC_FREQUENCY = freq;

            // _motor1.motorS.attach(motor1_pin);
            // _motor2.motorS.attach(motor2_pin);
            // _motor3.motorS.attach(motor3_pin);
            // _motor4.motorS.attach(motor4_pin);
            // _motor1.pin = motor1_pin;
            // _motor1.PWM_val = MOTOR_OFF;
            // _motor2.pin = motor2_pin;
            // _motor2.PWM_val = MOTOR_OFF;
            // _motor3.pin = motor3_pin;
            // _motor3.PWM_val = MOTOR_OFF;
            // _motor4.pin = motor4_pin;
            // _motor4.PWM_val = MOTOR_OFF;

            _motor1 = {motor1_pin, MOTOR_OFF};
            _motor2 = {motor2_pin, MOTOR_OFF};
            _motor3 = {motor3_pin, MOTOR_OFF};
            _motor4 = {motor4_pin, MOTOR_OFF};

        };

        Motor_t* motor1 = &_motor1;
        Motor_t* motor2 = &_motor2;
        Motor_t* motor3 = &_motor3;
        Motor_t* motor4 = &_motor4;


        void motor_calibration();
        //void ESC_ControllerCalibration(int throttle);
        void setup_pins();
        void Initialization(int, int);
        void Arm();
        void Disarm();

        // Getters and Setters
        int get_motor_pin(Motor_t motor);
        int get_motor_PWM(Motor_t motor);
        void set_motor_pin(Motor_t motor, int pin);
        void set_motorPWM(Motor_t* motor, int PWM);

};


#endif