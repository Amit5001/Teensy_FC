#include <SPI.h>
#include "Bitcraze_PMW3901.h"

#define CS_PIN 15  // שנה בהתאם לצורך
#define HEIGHT_METERS 1.5
#define PIXELS_PER_METER 1250.0  // בהתאם לניסוי/כיול
#define READ_INTERVAL_MS 10

Bitcraze_PMW3901 pmw(CS_PIN);

unsigned long lastReadTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!pmw.begin()) {
    Serial.println("Failed to initialize PMW3901");
    while (1);
  }
  delay(1000);
  Serial.println("PMW3901 initialized");
}

void loop() {
  static int16_t deltaX, deltaY;

  unsigned long now = millis();
  if (now - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = now;

    pmw.readMotionCount(&deltaX, &deltaY);

    // חישוב מהירות קווית במטרים לשנייה
    float intervalSec = READ_INTERVAL_MS / 1000.0;
    float velocityX = (float)deltaX / PIXELS_PER_METER / intervalSec;
    float velocityY = (float)deltaY / PIXELS_PER_METER / intervalSec;

    Serial.print("Velocity X: ");
    Serial.print(velocityX, 4);
    Serial.print(" m/s, Y: ");
    Serial.print(velocityY, 4);
    Serial.println(" m/s");
  }
  delay(500);

}