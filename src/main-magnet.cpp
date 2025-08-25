#include <Wire.h>
#include <Adafruit_LIS2MDL.h>
#include <math.h>

#define SDA_PIN 5
#define SCL_PIN 4

Adafruit_LIS2MDL lis2mdl;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lis2mdl.begin(0x1E, &Wire)) {
    Serial.println("Could not find LIS2MDLTR!");
    while (1) delay(10);
  }
  Serial.println("LIS2MDLTR found!");
}
// ...existing code...
// Add these at the top to store offsets
float x_offset = -73.725;
float y_offset = -3.675;
float max_x = -100;
float min_x = 100;
float max_y = -10;
float min_y = 100;

void loop() {
  sensors_event_t event;
  lis2mdl.getEvent(&event);

  float x = event.magnetic.x - x_offset;
  float y = event.magnetic.y - y_offset;
  if (x > max_x) max_x = x;
  if (x < min_x) min_x = x;
  if (y > max_y) max_y = y;  
  if (y < min_y) min_y = y;

  float heading = atan2(y, x) * 180.0 / PI;
  heading -= 310.0;
  if (heading < 0) heading += 360.0;
  if (heading < 0) heading += 360.0;
  if (heading > 360) heading -= 360.0;


  Serial.print("X: "); Serial.print(x);
  Serial.print("  Y: "); Serial.print(y);
  Serial.print("  Z: "); Serial.print(event.magnetic.z);
  Serial.print("  Heading: "); Serial.print(heading, 1); Serial.println(" deg");
//   Serial.print("X range: "); Serial.print(min_x); Serial.print(" to "); Serial.print(max_x);
//   Serial.print("  Y range: "); Serial.print(min_y); Serial.print(" to "); Serial.println(max_y);

  delay(500);
}
// ...existing code...