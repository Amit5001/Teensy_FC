#include <Arduino.h>
#include <02Motor/Motor_type.h>
#include <Servo.h>



void Motors::setup_pins(){
    pinMode(_motor1.pin, OUTPUT);
    pinMode(_motor2.pin, OUTPUT);
    pinMode(_motor3.pin, OUTPUT);
    pinMode(_motor4.pin, OUTPUT);
    analogWriteFrequency(_motor1.pin, ESC_FREQUENCY);
    analogWriteResolution(16);
    analogWriteFrequency(_motor2.pin, ESC_FREQUENCY);
    analogWriteResolution(16);  
    analogWriteFrequency(_motor3.pin, ESC_FREQUENCY);
    analogWriteResolution(16);
    analogWriteFrequency(_motor4.pin, ESC_FREQUENCY);
    analogWriteResolution(16);
    Serial.println("Setting up Motor Pins for PWM");
    Serial.println("Frequency: "); Serial.println(ESC_FREQUENCY);
    delay(1000);
}

void Motors::set_motorPWM(Motor_t* motor, int PWM_val){
    int PWM = map(PWM_val,MOTOR_MIN,MOTOR_MAX,0,PWM_MAX);
    motor->PWM_val = PWM_val;
    //analogWrite(motor->pin, PWM_val*4096/4000);
    analogWrite(motor->pin, PWM);
    Serial.print("Setting Motor "); Serial.print(motor->pin); Serial.print(" to "); Serial.println(PWM_val);
}

void Motors::motor_calibration(){
    Serial.println("Starting Motor Calibration");
    set_motorPWM(&_motor1, MOTOR_MAX);
    set_motorPWM(&_motor2, MOTOR_MAX);
    set_motorPWM(&_motor3, MOTOR_MAX);
    set_motorPWM(&_motor4, MOTOR_MAX);
    delay(15000);
    set_motorPWM(&_motor1, MOTOR_MIN);
    set_motorPWM(&_motor2, MOTOR_MIN);
    set_motorPWM(&_motor3, MOTOR_MIN);
    set_motorPWM(&_motor4, MOTOR_MIN);
    delay(10000);
    Serial.println("Motor Calibration Complete");
}

void Motors::Initialization(int calibrate, int contoller_calibration){
    setup_pins();
    if (calibrate == 1){
        motor_calibration();
    }
    // Serial.print("_Motor 1 Pin: "); Serial.println(_motor1.pin);
    // Serial.print("_Motor 1 PWM: "); Serial.println(_motor1.PWM_val);

}

void Motors::Arm(){
    Serial.println("Arming Motors");
    delay(500);
    set_motorPWM(&_motor1, MOTOR_ARM);
    set_motorPWM(&_motor2, MOTOR_ARM);
    set_motorPWM(&_motor3, MOTOR_ARM);
    set_motorPWM(&_motor4, MOTOR_ARM);
    delay(100);
    Serial.println("Motors Armed");
}

void Motors::Disarm(){
    Serial.println("Disarming Motors");
    delay(500);
    set_motorPWM(&_motor1, MOTOR_OFF);
    set_motorPWM(&_motor2, MOTOR_OFF);
    set_motorPWM(&_motor3, MOTOR_OFF);
    set_motorPWM(&_motor4, MOTOR_OFF);
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

