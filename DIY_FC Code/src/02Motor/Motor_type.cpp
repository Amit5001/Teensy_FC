#include <Arduino.h>
#include <Motor_type.h>
#include <Servo.h>



void Motors::setup_pins(){
    analogWriteFrequency(motor1.pin, ESC_FREQUENCY);
    analogWriteFrequency(motor2.pin, ESC_FREQUENCY);
    analogWriteFrequency(motor3.pin, ESC_FREQUENCY);
    analogWriteFrequency(motor4.pin, ESC_FREQUENCY);
    analogWriteResolution(12);
    delay(250);
}

void Motors::set_motor(Motor_t motor, int PWM_val){
    int PWM = map(PWM_val,MOTOR_MIN,MOTOR_MAX,0,4095);
    motor.PWM_val = PWM_val;
    analogWrite(motor.pin, PWM);
}

void Motors::motor_calibration(){
    Serial.println("Starting Motor Calibration");
    delay(1000);
    set_motor(motor1, MOTOR_MAX);
    set_motor(motor2, MOTOR_MAX);
    set_motor(motor3, MOTOR_MAX);
    set_motor(motor4, MOTOR_MAX);
    delay(2000);
    set_motor(motor1, MOTOR_MIN);
    set_motor(motor2, MOTOR_MIN);
    set_motor(motor3, MOTOR_MIN);
    set_motor(motor4, MOTOR_MIN);
    Serial.println("Motor Calibration Complete");
}

void Motors::Initialization(){
    setup_pins();
    motor_calibration();
}

void Motors::Arm(){
    Serial.println("Arming Motors");
    delay(500);
    set_motor(motor1, MOTOR_ARM);
    set_motor(motor2, MOTOR_ARM);
    set_motor(motor3, MOTOR_ARM);
    set_motor(motor4, MOTOR_ARM);
    delay(100);
    Serial.println("Motors Armed");
}

void Motors::Disarm(){
    Serial.println("Disarming Motors");
    delay(500);
    set_motor(motor1, MOTOR_OFF);
    set_motor(motor2, MOTOR_OFF);
    set_motor(motor3, MOTOR_OFF);
    set_motor(motor4, MOTOR_OFF);
    delay(100);
    Serial.println("Motors Disarmed");
}


// Getters and Setters
int Motors::get_motor_pin(Motor_t motor){
    return motor.pin;
}

int Motors::get_motor_PWM(Motor_t motor){
    return motor.PWM_val;
}

void Motors::set_motor_pin(Motor_t motor, int pin){
    motor.pin = pin;
}

void Motors::set_motor_PWM(Motor_t motor, int PWM){
    motor.PWM_val = PWM;
}
