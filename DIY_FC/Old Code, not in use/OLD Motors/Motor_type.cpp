#include <Arduino.h>
#include <02Motor/Motor_type.h>
#include <Servo.h>



void Motors::setup_pins(){
    _motor1.motorS.attach(_motor1.pin,PWM_MIN,PWM_MAX);
    _motor2.motorS.attach(_motor2.pin,PWM_MIN,PWM_MAX);
    _motor3.motorS.attach(_motor3.pin,PWM_MIN,PWM_MAX);
    _motor4.motorS.attach(_motor4.pin,PWM_MIN,PWM_MAX);
    delay(1000);
}

void Motors::set_motorPWM(Motor_t* motor, int PWM_val){
    motor->PWM_val = PWM_val;
    motor->motorS.writeMicroseconds(PWM_val);
    //Serial.print("Setting Motor "); Serial.print(motor->pin); Serial.print(" to "); Serial.println(PWM_val);
}

void Motors::ESC_calibration(){
    Serial.println("Starting Motor Calibration");
    set_motorPWM(&_motor1, PWM_MAX);
    set_motorPWM(&_motor2, PWM_MAX);
    set_motorPWM(&_motor3, PWM_MAX);
    set_motorPWM(&_motor4, PWM_MAX);
    delay(15000);
    set_motorPWM(&_motor1, PWM_MIN);
    set_motorPWM(&_motor2, PWM_MIN);
    set_motorPWM(&_motor3, PWM_MIN);
    set_motorPWM(&_motor4, PWM_MIN);
    delay(10000);
    Serial.println("Motor Calibration Complete");
}

void Motors::Initialization(int calibrate, int contoller_calibration){
    setup_pins();
    if (calibrate == 1){
        ESC_calibration();
    }
    // if (contoller_calibration == 1){
    //     ESC_ControllerCalibration(1500); // Need to change to a variable that contains the controller value.
    // }

}

// void Motors::ESC_ControllerCalibration(int throttle){
//     Serial.println("Starting Controller Calibration");
//     set_motorPWM(&_motor1, throttle);
//     set_motorPWM(&_motor2, throttle);
//     set_motorPWM(&_motor3, throttle);
//     set_motorPWM(&_motor4, throttle);
//     delay(5000);
//     Serial.println("Controller Calibration Complete");
// }

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

