#include <Arduino.h>


float Ax, Ay, Az;
float Gx, Gy, Gz;

double roll = 0.00;
double pitch = 0.00;

double roll_f = 0.00;
double pitch_f = 0.00;

double roll_f_m1 = 0.00;
double pitch_f_m1 = 0.00;

double roll_prediction = 0.00;
double pitch_prediction = 0.00;

double roll_f_error = 0.00;
double pitch_f_error = 0.00;

double roll_f_error_int = 0.00;
double pitch_f_error_int = 0.00;

double roll_f_error_int_m1 = 0.00;
double pitch_f_error_int_m1 = 0.00;

float time_now = 0;
float sec_count_ind = 1;
unsigned long sec_ = 1;

const float dt = 0.1; // in sec.
const float loop_period = dt * 1000; // in milli sec.
const float rad2deg = 57.2958;
const float sec_count = 1/dt;

const float tau = 0.2; // filter time constant
const float L_P = (dt/tau) / (1 + dt/tau);
const float L_I = L_P/4;

const float L_P_roll = L_P;
const float L_I_roll = L_I;
const float roll_int_d_factor = 0.3;

const float L_P_pitch = L_P;
const float L_I_pitch = L_I;
const float pitch_int_d_factor = 0.3;


void setup() {
  Serial.begin(9600);
  //Initializing OpenLog
  Serial1.begin(9600); // Start serial communication with OpenLog
  delay(1000); // Delay to allow OpenLog to initialize  
  Serial.println("Initializing SparkFun OpenLog...");
  Serial.println("Done Initializing OpenLog");
  while(!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  Serial.println();

  Serial.print("Gyroscope sample rate = ");  
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println("Hz");
  Serial.println();


}

void loop() {

  time_now = millis();

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);

    // Serial.println("Accelerometer data: ");
    // Serial.print(Ax);
    // Serial.print('\t');
    // Serial.print(Ay);
    // Serial.print('\t');
    // Serial.println(Az);
    // Serial.println();
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gx, Gy, Gz);
    
    // Serial.println("Gyroscope data: ");
    // Serial.print(Gx);
    // Serial.print('\t');
    // Serial.print(Gy);
    // Serial.print('\t');
    // Serial.println(Gz);
    // Serial.println();
  }

//Calculation of the different inclinations (pitch, roll)
  roll = atan2(Ay, Az) * rad2deg;
  pitch = atan2((-Ax), sqrt(Ay*Ay + Az*Az)) * rad2deg;


  roll_prediction = roll_f_m1 + Gx*dt;
  roll_f_error = roll - roll_prediction;
  roll_f_error_int = roll_int_d_factor * roll_f_error_int_m1 + roll_f_error;
  roll_f = roll_prediction + L_P_roll * roll_f_error + L_I_roll * roll_f_error_int;
  roll_f_m1 = roll_f;
  roll_f_error_int_m1 = roll_f_error_int;

  pitch_prediction = pitch_f_m1 + Gy*dt;
  pitch_f_error = pitch - pitch_prediction;
  pitch_f_error_int = pitch_int_d_factor * pitch_f_error_int_m1 + pitch_f_error;
  pitch_f = pitch_prediction + L_P_pitch * pitch_f_error + L_I_pitch * pitch_f_error_int;
  pitch_f_m1 = pitch_f;
  pitch_f_error_int_m1 = pitch_f_error_int;

  // Serial.print(roll);
  // Serial.print('\t');
  // Serial.println(roll_f);

  Serial.print(pitch);
  Serial.print('\t');
  Serial.println(pitch_f);

  
  while(millis() < time_now + loop_period){
        //wait approx. [loop_period] ms
    }

}
