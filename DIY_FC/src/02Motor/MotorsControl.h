#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <Arduino.h>
#include <Var_types.h>

class Motors {
    private:
        // int PWM_MIN = 1100;
        // int PWM_MAX = 2000;
        int MOTOR_OFF = 1000;
        int MOTOR_ARM = 1100;
        int ESC_FREQUENCY;

        motor_t Motor_struct;

    public:
        Motors(int freq, int motor1_pin, int motor2_pin, int motor3_pin, int motor4_pin){
            ESC_FREQUENCY = freq;
            int PWM_RESOLUTION = 12; // 12-bit resolution
            int PWM_PERIOD = 1000000/ESC_FREQUENCY; // Period in microseconds
            int PWM_MAX = ((1 << PWM_RESOLUTION) - 1)  // 4095 for 12-bit
            int PWM_MIN = (1000*PWM_MAX) / (PWM_PERIOD);

            // Calculating the PWM boundaries- they change according to the frequency and the resolution
            int PWM_MIN_US = 1000;
            int PWM_MAX_US = 2000;
            int PWM_MAX_MAP = PWM_MAX_US * PWM_MAX/ PWM_PERIOD // Conversion of 2000 msec to the PWM duty cycle value
            int PWM_MIN_MAP = PWM_MIN_US * PWM_MIN/ PWM_PERIOD // Conversion of 1000 msec to the PWM duty cycle value

            #define US_2_PULSE(u) ((u)* PWM_MAX / (PWM_PERIOD)) // This function converts the microsecond value to the PWM duty cycle value
            // Based on the define and PWM_MAX_MAP, PWM_MIN_MAP, we can convert controller data to the PWM duty cycle value. CHOOSE ONLY ONE OPTION!

            Motor_struct.M1_pin = motor1_pin;
            Motor_struct.M2_pin = motor2_pin;
            Motor_struct.M3_pin = motor3_pin;
            Motor_struct.M4_pin = motor4_pin;
            Motor_struct.PWM1 = MOTOR_OFF;
            Motor_struct.PWM2 = MOTOR_OFF;
            Motor_struct.PWM3 = MOTOR_OFF;
            Motor_struct.PWM4 = MOTOR_OFF;


        };
        void Motors_init();
        void Arm();
        void Disarm();
        void set_motorPWM();
        void Motor_Mix(attitude_t , int );
        motor_t Get_motor();

};

#endif // _MOTORS_CONTROL_H_