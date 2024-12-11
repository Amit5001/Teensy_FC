/***************************************************************************
 * Example sketch for the Teensy-DShot library
 *
 * This example initiates the ESCs, increases the throttle to maximum speed,
 * and then decreases the throttle to zero.
 *
 ***************************************************************************/
#include <Arduino.h>

#include "DShot.h"

constexpr uint16_t LOOP_HZ = 2000;

DShot motor0(&Serial2, DShotType::DShot600); // Teensy4.X Pin 8


void setup()
{
    // Setup blink led
    pinMode(LED_BUILTIN, OUTPUT);

    // Wait for motor init
    for (size_t i = 0; i < 4000; i++) {
        motor0.sendCommand(0, false);
        // Serial.println("Init motor");
        delayMicroseconds(1'000);
    }
    // motor0.sendThrottle(0, false);
    // delayMicroseconds(1'000);
}

uint64_t counter = 0;
int16_t throttle = 0;
int8_t throttleChange = 1;
uint8_t ledState = false;

void loop()
{
    // motor0.sendThrottle(0, false);
    // Serial.println("Throttle 0");
    uint32_t loopCycleStart = micros();

    // motor0.sendThrottle(0, false);
    // Serial.println("Throttle 2000");
    // Set throttle
    motor0.sendThrottle(throttle, false);
    Serial.println(throttle);

    // Decrease throttle if max reached
    if (throttle >= 1999) {
        throttleChange = -1;
    }

    // Increase or decrease throttle
    if (counter % 20 == 0 && !(throttle == 0 && throttleChange == -1)) {
        throttle += throttleChange;
    }

    // Blink the LED
    if (counter % LOOP_HZ == 0) {
        digitalWrite(LED_BUILTIN, ledState ^= 1);
    }

    ++counter;
    delayMicroseconds((1'000'000 / LOOP_HZ) - (micros() - loopCycleStart));
}