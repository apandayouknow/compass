// ...existing code...
#include <Wire.h>
#include <Adafruit_LIS2MDL.h>
#include <math.h>
#include <Preferences.h>

#define SDA_PIN 5
#define SCL_PIN 4

Adafruit_LIS2MDL lis2mdl;
Preferences prefs;

/*
21/11
float x_offset = -62.78;
float y_offset = -21.08;
float scale_x = 0.95;
float scale_y = 1.06;

21/11 better
float x_offset = -64.80;
float y_offset = -20.33;
float scale_x = 0.98;
float scale_y = 1.02;

*/

// calibration state (defaults from your file, will be overwritten by calibration)
float x_offset = -59.425;
float y_offset = -8.5;
float scale_x = 1.0;
float scale_y = 1.0;
float heading_offset = 0.0;

// float x_offset = -62.78;
// float y_offset = -21.08;
// float scale_x = 0.95;
// float scale_y = 1.06;
// float heading_offset = 250;

float max_x = -100;
float min_x = 100;
float max_y = -10;
float min_y = 100;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lis2mdl.begin(0x1E, &Wire)) {
    Serial.println("Could not find LIS2MDLTR!");
    while (1) delay(10);
  }
  Serial.println("LIS2MDLTR found!");
  Serial.println("Rotate board slowly for calibration. Send 'c' over Serial to calibrate.");

  prefs.begin("magcal", false);
  x_offset = prefs.getFloat("x_offset", x_offset);
  y_offset = prefs.getFloat("y_offset", y_offset);
  scale_x  = prefs.getFloat("scale_x", scale_x);
  scale_y  = prefs.getFloat("scale_y", scale_y);
  heading_offset = prefs.getFloat("heading_offset", heading_offset);
}

// Calibrate magnetometer for 'seconds' seconds. Rotate board slowly around all axes.
void calibrateMag(unsigned long seconds = 20) {
  Serial.println("Starting calibration: rotate device slowly in all orientations...");
  unsigned long start = millis();
  // initialize min/max with current reading
  sensors_event_t ev;
  lis2mdl.getEvent(&ev);
  min_x = max_x = ev.magnetic.x;
  min_y = max_y = ev.magnetic.y;

  while (millis() - start < seconds * 1000UL) {
    lis2mdl.getEvent(&ev);
    float x = ev.magnetic.x;
    float y = ev.magnetic.y;
    if (x > max_x) max_x = x;
    if (x < min_x) min_x = x;
    if (y > max_y) max_y = y;
    if (y < min_y) min_y = y;
    delay(50); // sample ~20Hz
  }

  // compute offsets (hard-iron)
  x_offset = (max_x + min_x) / 2.0;
  y_offset = (max_y + min_y) / 2.0;

  // compute scale factors to normalize radii (soft-iron)
  float rx = (max_x - min_x) / 2.0;
  float ry = (max_y - min_y) / 2.0;
  float r_avg = (rx + ry) / 2.0;
  if (rx == 0) scale_x = 1.0; else scale_x = r_avg / rx;
  if (ry == 0) scale_y = 1.0; else scale_y = r_avg / ry;

  Serial.println("Calibration complete:");
  Serial.print("min_x="); Serial.print(min_x); Serial.print(" max_x="); Serial.println(max_x);
  Serial.print("min_y="); Serial.print(min_y); Serial.print(" max_y="); Serial.println(max_y);
  Serial.print("x_offset="); Serial.println(x_offset, 6);
  Serial.print("y_offset="); Serial.println(y_offset, 6);
  Serial.print("scale_x="); Serial.println(scale_x, 6);
  Serial.print("scale_y="); Serial.println(scale_y, 6);

  prefs.putFloat("x_offset", x_offset);
  prefs.putFloat("y_offset", y_offset);
  prefs.putFloat("scale_x", scale_x);
  prefs.putFloat("scale_y", scale_y);
  prefs.putFloat("heading_offset", heading_offset);
  Serial.println("Calibration saved to flash.");

  Serial.println("Apply these offsets/scales in code or keep device moving for further refinement.");
}

// Apply calibration to raw x,y
void applyCalibration(float raw_x, float raw_y, float &out_x, float &out_y) {
  out_x = (raw_x - x_offset) * scale_x;
  out_y = (raw_y - y_offset) * scale_y;
}

void loop() {
  // allow calibration trigger from serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      calibrateMag(40); // calibrate for 40 seconds
    } else if (c == 'r' || c == 'R') {
      // clear stored calibration and reset to defaults (or current defaults in code)
      prefs.clear();
      Serial.println("Stored calibration cleared. Restart device or re-run to use defaults.");
    }
  }

  sensors_event_t event;
  lis2mdl.getEvent(&event);

  // raw values
  float raw_x = event.magnetic.x;
  float raw_y = event.magnetic.y;
  float raw_z = event.magnetic.z;

  // apply calibration
  float x, y;
  applyCalibration(raw_x, raw_y, x, y);

  // compute heading: atan2(y, x) -> 0 = +x, 90 = +y. Adjust as needed for your board orientation.
  float heading = atan2(x, y) * 180.0 / PI;
  heading += heading_offset;
  if (heading < 0) heading += 360.0;
  if (heading >= 360.0) heading -= 360.0;

  // update ranges (optional)
  if (raw_x > max_x) max_x = raw_x;
  if (raw_x < min_x) min_x = raw_x;
  if (raw_y > max_y) max_y = raw_y;  
  if (raw_y < min_y) min_y = raw_y;

  Serial.print("Raw X: "); Serial.print(raw_x, 3);
  Serial.print("  Raw Y: "); Serial.print(raw_y, 3);
  Serial.print("  Cal X: "); Serial.print(x, 3);
  Serial.print("  Cal Y: "); Serial.print(y, 3);
  Serial.print("  Z: "); Serial.print(raw_z, 3);
  Serial.print("  Heading: "); Serial.print(heading, 1); Serial.println(" deg");

  Serial.print("X range: "); Serial.print(min_x,3); Serial.print(" to "); Serial.print(max_x,3);
  Serial.print("  Y range: "); Serial.print(min_y,3); Serial.print(" to "); Serial.println(max_y,3);

  Serial.print("X offset: "); Serial.print(x_offset); Serial.print(" Y offset: "); Serial.println(y_offset);
  Serial.print("X scale: "); Serial.print(scale_x); Serial.print(" Y scale: "); Serial.println(scale_y);

  delay(200);
}
// ...existing code...