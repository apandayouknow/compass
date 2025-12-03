/*
init
helper functions 
setup 

calibration:
- 1 short press to get to calibration screen 
- 1 long press within cooldown to point north 
- 1 short press within cooldown to begin calibration 

main loop:
- (only run one process every loop)
- if button press run calibration 
- alternate check for gps flag 
- and check for changes in direction 
- if change in either direction or gps flag, render arrow 

*/

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <Adafruit_LIS2MDL.h>
#include <math.h>
#include <Preferences.h>

//Magnetometer pins
#define SDA_PIN 5
#define SCL_PIN 4

//Devices
Adafruit_LIS2MDL lis2mdl;
TFT_eSPI tft = TFT_eSPI();
SoftwareSerial gpsSerial(3,2);

//Auto coordinates
double target_lat = 1.3519688;
double target_lon = 103.848658;

//Flash vars
Preferences prefs;
float x_offset = 0.0;
float y_offset = 0.0;
float scale_x = 1.0;
float scale_y = 1.0;
float heading_offset = 0.0;

//Calibration vars
float max_x = -100;
float min_x = 100;
float max_y = -10;
float min_y = 100;

//Loop vars
bool gpsFlag, compassFlag, button; 
bool alternate;
float currHeading, lastHeading;
float longitude, latitude;

//GPS helper functions
static const char* fieldStart(const char *line, int n) {
  // return pointer to start of nth field (1 = first field after sentence id)
  const char *p = strchr(line, ',');
  if (!p) return NULL;
  p++;
  for (int i = 1; i < n; ++i) {
    p = strchr(p, ',');
    if (!p) return NULL;
    p++;
  }
  return p;
}

static bool extractField(const char *line, int n, char *out, size_t outSize) {
  const char *s = fieldStart(line, n);
  if (!s) return false;
  const char *e = strchr(s, ',');
  if (!e) {
    const char *star = strchr(s, '*');
    if (star) e = star;
    else e = s + strlen(s);
  }
  size_t len = (size_t)(e - s);
  if (len >= outSize) len = outSize - 1;
  memcpy(out, s, len);
  out[len] = '\0';
  return true;
}

static double dmToDeg(const char *dmStr) {
  if (!dmStr || dmStr[0] == '\0') return NAN;
  double v = atof(dmStr);
  int deg = (int)(v / 100);
  double minutes = v - deg * 100;
  return deg + minutes / 60.0;
}

static bool parseGGA(const char *line, double &lat, double &lon) {
  // GGA fields: 1=time,2=lat,3=N/S,4=lon,5=E/W,6=fix
  char buf[32];
  if (!extractField(line, 6, buf, sizeof(buf))) return false;
  if (buf[0] == '\0' || buf[0] == '0') return false; // no fix (0) or empty
  if (!extractField(line, 2, buf, sizeof(buf))) return false;
  double latd = dmToDeg(buf);
  extractField(line, 3, buf, sizeof(buf));
  if (buf[0] == 'S') latd = -latd;
  if (!extractField(line, 4, buf, sizeof(buf))) return false;
  double lond = dmToDeg(buf);
  extractField(line, 5, buf, sizeof(buf));
  if (buf[0] == 'W') lond = -lond;
  if (isnan(latd) || isnan(lond)) return false;
  lat = latd; lon = lond;
  return true;
}

static bool parseRMC(const char *line, double &lat, double &lon) {
  // RMC fields: 1=time,2=status(A/V),3=lat,4=N/S,5=lon,6=E/W
  char buf[32];
  if (!extractField(line, 2, buf, sizeof(buf))) return false;
  if (buf[0] != 'A') return false; // A = valid
  if (!extractField(line, 3, buf, sizeof(buf))) return false;
  double latd = dmToDeg(buf);
  extractField(line, 4, buf, sizeof(buf));
  if (buf[0] == 'S') latd = -latd;
  if (!extractField(line, 5, buf, sizeof(buf))) return false;
  double lond = dmToDeg(buf);
  extractField(line, 6, buf, sizeof(buf));
  if (buf[0] == 'W') lond = -lond;
  if (isnan(latd) || isnan(lond)) return false;
  lat = latd; lon = lond;
  return true;
}

static bool parseGLL(const char *line, double &lat, double &lon) {
  // GLL fields: 1=lat,2=N/S,3=lon,4=E/W,5=time,6=status(A/V)
  char buf[32];
  if (!extractField(line, 6, buf, sizeof(buf))) return false;
  if (buf[0] != 'A') return false; // A = valid
  if (!extractField(line, 1, buf, sizeof(buf))) return false;
  double latd = dmToDeg(buf);
  extractField(line, 2, buf, sizeof(buf));
  if (buf[0] == 'S') latd = -latd;
  if (!extractField(line, 3, buf, sizeof(buf))) return false;
  double lond = dmToDeg(buf);
  extractField(line, 4, buf, sizeof(buf));
  if (buf[0] == 'W') lond = -lond;
  if (isnan(latd) || isnan(lond)) return false;
  lat = latd; lon = lond;
  return true;
}

static bool extractCoordsFromLine(const char *line, double &lat, double &lon) {
  const char *candidates[] = { "$GNGGA", "$GPGGA", "$GNRMC", "$GPRMC", "$GNGLL", "$GPGLL" };
  char buf[128];
  for (size_t i = 0; i < sizeof(candidates)/sizeof(candidates[0]); ++i) {
    const char *s = strstr(line, candidates[i]);
    if (!s) continue;
    // find end of sentence (newline or '*' or end)
    const char *e = strchr(s, '\n');
    const char *star = strchr(s, '*');
    if (!e || (star && star < e)) e = star;
    if (!e) e = s + strlen(s);
    size_t len = (size_t)(e - s);
    if (len >= sizeof(buf)) continue;
    memcpy(buf, s, len);
    buf[len] = '\0';
    // call appropriate parser
    if (strncmp(buf, "$GNGGA", 6) == 0 || strncmp(buf, "$GPGGA", 6) == 0) {
      if (parseGGA(buf, lat, lon)) return true;
    } else if (strncmp(buf, "$GNRMC", 6) == 0 || strncmp(buf, "$GPRMC", 6) == 0) {
      if (parseRMC(buf, lat, lon)) return true;
    } else if (strncmp(buf, "$GNGLL", 6) == 0 || strncmp(buf, "$GPGLL", 6) == 0) {
      if (parseGLL(buf, lat, lon)) return true;
    }
  }
  return false;
}


//Calibration function
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
  Serial.println("Calibration saved to flash.");

  Serial.println("Apply these offsets/scales in code or keep device moving for further refinement.");
}

//Apply calibration to raw x,y
void applyCalibration(float raw_x, float raw_y, float &out_x, float &out_y) {
  out_x = (raw_x - x_offset) * scale_x;
  out_y = (raw_y - y_offset) * scale_y;
}

//Heading helper
float getHeading(){
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
  while (heading < 0) {
    heading += 360.0;
  }
  while (heading > 360.0){
    heading -= 360.0;
  }
  return heading;
}
//Set heading
void calibrateNorth(){
  heading_offset += getHeading();
  prefs.putFloat("heading_offset", heading_offset);
}
//Calibrate function
void calibrateScreen(){
  Serial.println("Short press to calibrate, long press to set direction");
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(20,50,2);
  tft.println("Short press to calibrate");
  tft.setCursor(20,70,2);
  tft.println("Long press to set North");
  delay(500);
  unsigned long shortThresh = 1;
  while (digitalRead(9) != false);
  unsigned long start = millis();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(20,50,2);
  while (digitalRead(9) != true);
  if (millis() - start < shortThresh * 1000UL){
    tft.println("Rotate slowly in all directions");
    calibrateMag();
  }
  else {
    tft.println("Point the device North");
    delay(200);
    calibrateNorth();
  }
  
}
//Render helper
void render(float heading, float longitude, float latitude){
  float calcx = cos(target_lat * PI / 180.0) * sin((longitude - target_lon) * PI / 180.0);
  float calcy = cos(latitude * PI / 180.0) * sin(target_lat * PI / 180.0) - sin(latitude * PI / 180.0) * cos(target_lat * PI / 180.0) * cos((longitude - target_lon) * PI / 180.0);
  float bearing = atan2(calcx, calcy) * 180.0 / PI;
  // bearing += 180.0;
  bearing -= heading;
  while (bearing < 0){
    bearing += 360.0;
  }
  while (bearing > 360){
    bearing -= 360.0;
  }
  Serial.print("Bearing");
  Serial.println(bearing);

  // clear screen
  tft.fillScreen(TFT_BLACK);

  // arrow geometry
  int cx = tft.width() / 2;
  int cy = tft.height() / 2;
  int len = min(tft.width(), tft.height()) / 3;        // arrow length
  int headLen = max(8, len / 5);                      // head length
  int shaftWidth = max(2, len / 30);
  int halfHeadW = max(6, len / 12);

  // convert bearing to screen angle: 0° (north) -> up on screen
  float angle = (bearing - 90.0) * PI / 180.0;

  // tip point
  int tipX = cx + (int)(len * cos(angle));
  int tipY = cy + (int)(len * sin(angle));

  // base of head (where triangle base sits)
  int baseX = cx + (int)((len - headLen) * cos(angle));
  int baseY = cy + (int)((len - headLen) * sin(angle));

  // perpendicular for head base width
  float perp = angle + PI / 2.0;
  int leftX  = baseX + (int)(halfHeadW * cos(perp));
  int leftY  = baseY + (int)(halfHeadW * sin(perp));
  int rightX = baseX - (int)(halfHeadW * cos(perp));
  int rightY = baseY - (int)(halfHeadW * sin(perp));

  // draw shaft (thick by drawing multiple parallel lines)
  for (int w = -shaftWidth; w <= shaftWidth; ++w) {
    int sx1 = cx + (int)( (w * cos(perp)) );
    int sy1 = cy + (int)( (w * sin(perp)) );
    int sx2 = baseX + (int)( (w * cos(perp)) );
    int sy2 = baseY + (int)( (w * sin(perp)) );
    tft.drawLine(sx1, sy1, sx2, sy2, TFT_WHITE);
  }

  // draw arrow head
  tft.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, TFT_WHITE);

  // small center circle to mark origin
  tft.fillCircle(cx, cy, 4, TFT_WHITE);
}

void setup(){
  Serial.begin(115200);

  //Magnetometer init
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lis2mdl.begin(0x1E, &Wire)) {
    Serial.println("Could not find LIS2MDLTR!");
    while (1) delay(10);
  }
  Serial.println("LIS2MDLTR found!");

  //Get flash vars
  prefs.begin("magcal", false);
  x_offset = prefs.getFloat("x_offset", x_offset);
  y_offset = prefs.getFloat("y_offset", y_offset);
  scale_x  = prefs.getFloat("scale_x", scale_x);
  scale_y  = prefs.getFloat("scale_y", scale_y);
  heading_offset = prefs.getFloat("heading_offset", heading_offset);

  //GPS init
  gpsSerial.begin(9600);

  //Screen init
  tft.init();
  
  tft.fillScreen(TFT_BLACK);
  compassFlag = false;
  gpsFlag = false;
  button = false;

  currHeading = getHeading();
  lastHeading = currHeading;

  delay(5000);
}

void loop(){
  Serial.println("LOOP!");
  currHeading = getHeading();
  if (digitalRead(9) == false){
    Serial.println("BUTTON!");
    calibrateScreen();
  } else if (compassFlag == true || gpsFlag == true){
    Serial.println("RENDER!");
    render(currHeading - heading_offset, longitude, latitude);
    //lastHeading is heading of last render
    lastHeading = currHeading;
    compassFlag = false;
    gpsFlag = false;
  } else if (abs(currHeading - lastHeading) > 10){
    Serial.println("COMPASS FLAG!");
    compassFlag = true;
  } else {
    Serial.println("GPS loop");
    static char nmea[128];
    static uint8_t idx = 0;

    while (gpsSerial.available()) {
      yield(); // let RTOS/watchdog run
      int ci = gpsSerial.read();
      if (ci < 0) break;
      char c = (char)ci;
      Serial.write(c); // echo for debug

      if (c == '\n') {
        if (idx == 0) continue;
        nmea[idx] = '\0';
        double latv = NAN, lonv = NAN;
        if (extractCoordsFromLine(nmea, latv, lonv)) {
          latitude = (float)latv;
          longitude = (float)lonv;
          gpsFlag = 1;
          Serial.println(F("Got coords"));
        }
        idx = 0;
      } else if (c != '\r') {
        if (idx < (int)sizeof(nmea) - 1) nmea[idx++] = c;
        else idx = 0; // overflow -> discard
      }
    }
  }
  //throttle
  delay(200);
}
