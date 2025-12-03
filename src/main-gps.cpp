
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(3, 2); // RX, TX

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
}

void loop() {
  static char nmea[100];
  static byte idx = 0;

  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '\n') {
      nmea[idx] = 0;
      if (strstr(nmea, "$GNGGA") == nmea) {
        // Example: $GNGGA,091032.00,0120.76392,N,10350.47666,E,1,12,0.73,39.3,M,6.1,M,,*4A
        char *p = nmea;
        // Skip $GNGGA,
        p = strchr(p, ',') + 1; // time
        char timeStr[11] = {0};
        strncpy(timeStr, p, 10); // Copy up to next comma or 10 chars
        // Parse time
        int hour = (timeStr[0] - '0') * 10 + (timeStr[1] - '0');
        int min  = (timeStr[2] - '0') * 10 + (timeStr[3] - '0');
        int sec  = (timeStr[4] - '0') * 10 + (timeStr[5] - '0');
        Serial.print("UTC Time: ");
        Serial.print(hour);
        Serial.print(":");
        Serial.print(min);
        Serial.print(":");
        Serial.println(sec);
        p = strchr(p, ',') + 1; // latitude
        float lat = atof(p);
        p = strchr(p, ',') + 1; // N/S
        char ns = *p;
        p = strchr(p, ',') + 1; // longitude
        float lon = atof(p);
        p = strchr(p, ',') + 1; // E/W
        char ew = *p;

        // Convert latitude from ddmm.mmmm to decimal degrees
        int lat_deg = int(lat / 100);
        float lat_min = lat - lat_deg * 100;
        float latitude = lat_deg + lat_min / 60.0;
        if (ns == 'S') latitude = -latitude;

        // Convert longitude from dddmm.mmmm to decimal degrees
        int lon_deg = int(lon / 100);
        float lon_min = lon - lon_deg * 100;
        float longitude = lon_deg + lon_min / 60.0;
        if (ew == 'W') longitude = -longitude;

        Serial.print("Lat: ");
        Serial.print(latitude, 6);
        Serial.print("  Lon: ");
        Serial.println(longitude, 6);
      }
      idx = 0;
    } else if (idx < sizeof(nmea) - 1) {
      nmea[idx++] = c;
    }
  }
}


// // Echo GPS data to Serial Monitor
// #include <SoftwareSerial.h>
// SoftwareSerial gpsSerial(3, 2); // RX, TX

// void setup() {
//   Serial.begin(115200);
//   gpsSerial.begin(9600); // Try 38400 if nothing prints
// }

// void loop() {
//   while (gpsSerial.available()) {
//     Serial.write(gpsSerial.read());
//   }
// }


// // Minimal MAX-M10S GPS Example
// // Uses SoftwareSerial on pins 3 (RX) and 2 (TX)

// #include <SparkFun_u-blox_GNSS_v3.h>
// #include <SoftwareSerial.h>

// SoftwareSerial gpsSerial(3, 2); // RX, TX
// SFE_UBLOX_GNSS_SERIAL gps;

// void setup() {
//   Serial.begin(115200);
//   gpsSerial.begin(9600); // Most MAX-M10S modules default to 9600 baud

//   // Try to connect to the GPS module
//   while (!gps.begin(gpsSerial)) {
//     Serial.println("Waiting for GPS...");
//     delay(1000);
//   }
//   Serial.println("GPS connected!");
// }

// void loop() {
//   long lat = gps.getLatitude();
//   long lon = gps.getLongitude();

//   Serial.print("Lat: ");
//   Serial.print(lat / 10000000.0, 7);
//   Serial.print("  Lon: ");
//   Serial.println(lon / 10000000.0, 7);

//   delay(1000);
// }



// /*
//   Reading lat and long via UBX binary commands using UART @38400 baud - free from I2C
//   By: Nathan Seidle, Adapted from Example3_GetPosition by Thorsten von Eicken
//   SparkFun Electronics
//   Date: January 28rd, 2019
//   License: MIT. See license file for more information but you can
//   basically do whatever you want with this code.

//   This example shows how to configure the library and U-Blox for serial port use as well as
//   switching the module from the default 9600 baud to 38400.

//   Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
//   to something google maps understands simply divide the numbers by 10,000,000. We 
//   do this so that we don't have to use floating point numbers.

//   Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

//   Feel like supporting open source hardware?
//   Buy a board from SparkFun!
//   ZED-F9P RTK2: https://www.sparkfun.com/products/15136
//   NEO-M8P RTK: https://www.sparkfun.com/products/15005
//   SAM-M8Q: https://www.sparkfun.com/products/15106

//   Hardware Connections:
//   Connect the U-Blox serial TX pin to Uno pin 10
//   Connect the U-Blox serial RX pin to Uno pin 11
//   Open the serial monitor at 115200 baud to see the output
// */

// #include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
// SFE_UBLOX_GNSS_SERIAL myGNSS;

// #include <SoftwareSerial.h>
// SoftwareSerial mySerial(3,2); // RX, TX. Pin 10 on Uno goes to TX pin on GNSS module.

// long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to u-blox module.

// void setup()
// {
//   Serial.begin(115200);
//   while (!Serial); //Wait for user to open terminal
//   Serial.println("SparkFun u-blox Example");

//   //Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 38400 baud.
//   //Loop until we're in sync and then ensure it's at 38400 baud.
//   do {
//     Serial.println("GNSS: trying 38400 baud");
//     mySerial.begin(38400);
//     if (myGNSS.begin(mySerial) == true) break;

//     delay(100);
//     Serial.println("GNSS: trying 9600 baud");
//     mySerial.begin(9600);
//     if (myGNSS.begin(mySerial) == true) {
//         Serial.println("GNSS: connected at 9600 baud, switching to 38400");
//         myGNSS.setSerialRate(38400);
//         delay(100);
//     } else {
//         myGNSS.factoryReset();
//         delay(2000); //Wait a bit before trying again to limit the Serial output
//     }
//   } while(1);
// //   Serial.println("GNSS serial connected");

//   myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
//   myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
//   myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
// }

// void loop()
// {
//   //Query module only every second. Doing it more often will just cause I2C traffic.
//   //The module only responds when a new position is available
//   if (millis() - lastTime > 1000)
//   {
//     lastTime = millis(); //Update the timer
    
//     long latitude = myGNSS.getLatitude();
//     Serial.print(F("Lat: "));
//     Serial.print(latitude);

//     long longitude = myGNSS.getLongitude();
//     Serial.print(F(" Long: "));
//     Serial.print(longitude);
//     Serial.print(F(" (degrees * 10^-7)"));

//     long altitude = myGNSS.getAltitude();
//     Serial.print(F(" Alt: "));
//     Serial.print(altitude);
//     Serial.print(F(" (mm)"));

//     byte SIV = myGNSS.getSIV();
//     Serial.print(F(" SIV: "));
//     Serial.print(SIV);

//     Serial.println();
//   }
// }