
//   Diagnostic test for the displayed colour order
//
// Written by Bodmer 17/2/19 for the TFT_eSPI library:
// https://github.com/Bodmer/TFT_eSPI

/* 
 Different hardware manufacturers use different colour order
 configurations at the hardware level.  This may result in
 incorrect colours being displayed.

 Incorrectly displayed colours could also be the result of
 using the wrong display driver in the library setup file.

 Typically displays have a control register (MADCTL) that can
 be used to set the Red Green Blue (RGB) colour order to RGB
 or BRG so that red and blue are swapped on the display.

 This control register is also used to manage the display
 rotation and coordinate mirroring. The control register
 typically has 8 bits, for the ILI9341 these are:

 Bit Function
 7   Mirror Y coordinate (row address order)
 6   Mirror X coordinate (column address order)
 5   Row/column exchange (for rotation)
 4   Refresh direction (top to bottom or bottom to top in portrait orientation)
 3   RGB order (swaps red and blue)
 2   Refresh direction (top to bottom or bottom to top in landscape orientation)
 1   Not used
 0   Not used

 The control register bits can be written with this example command sequence:
 
    tft.writecommand(TFT_MADCTL);
    tft.writedata(0x48);          // Bits 6 and 3 set
    
 0x48 is the default value for ILI9341 (0xA8 for ESP32 M5STACK)
 in rotation 0 orientation.
 
 Another control register can be used to "invert" colours,
 this swaps black and white as well as other colours (e.g.
 green to magenta, red to cyan, blue to yellow).
 
 To invert colours insert this line after tft.init() or tft.begin():

    tft.invertDisplay( invert ); // Where invert is true or false

*/

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include <TFT_eSPI.h>       // Hardware-specific library

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

#define MAXTX 3
#define MAXRX 2


#include <SparkFun_u-blox_GNSS_v3.h> // Include the SparkFun u-blox GNSS library

SFE_UBLOX_GNSS_SERIAL myGNSS; // Create a u-blox GNSS object for serial communication

SoftwareSerial MaxSerial(MAXTX, MAXRX); // RX, TX
#define gpsSerial MaxSerial // Define the hardware serial port connected to the GPS module


void setup(void) {
  Serial.begin(9600);
  Serial.println("TFT_eSPI invertDisplay Example");
  tft.init();

  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_GREEN);

  // Set "cursor" at top left corner of display (0,0) and select font 4
  tft.setCursor(0, 4, 4);

  // Set the font colour to be white with a black background
  tft.setTextColor(TFT_WHITE);

  // We can now plot text on screen using the "print" class
  tft.println(" Initialised default\n");
  tft.println(" White text");
  
  tft.setTextColor(TFT_RED);
  tft.println(" Red text");
  
  tft.setTextColor(TFT_GREEN);
  tft.println(" Green text");
  
  tft.setTextColor(TFT_BLUE);
  tft.println(" Blue text");

  // Serial.begin(115200); // Initialize serial for debugging output
  delay(1000);
  Serial.println("Starting GPS...");

  gpsSerial.begin(9600); // Initialize the hardware serial port for the GPS module

  // Attempt to connect to the u-blox module
  while (myGNSS.begin(gpsSerial) == false) {
    Serial.println(F("u-blox GNSS not detected"));
    Serial.println(F("Attempting to enable the UBX protocol for output"));
    myGNSS.setUART1Output(COM_TYPE_UBX); // Enable UBX output on UART1
    Serial.println(F("Retrying..."));
    delay(1000);
  }
  Serial.println(F("u-blox GNSS detected!"));

  // Optional: Configure the module, e.g., set dynamic model for better performance
  myGNSS.setDynamicModel(DYN_MODEL_PORTABLE); // Set dynamic model to portable
  myGNSS.saveConfiguration(); // Save the configuration to flash

  delay(5000);

}


void loop() {
  // Serial.println("Testing MAX-M10S");
  if (myGNSS.getPVT() == true) {
    int32_t latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude); // Latitude in degrees * 10^-7

    int32_t longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude); // Longitude in degrees * 10^-7
    Serial.print(F(" (degrees * 10^-7)"));

    int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level in mm
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));
    Serial.println();
  }



  // Serial.println("Inverting display");

  // tft.invertDisplay( false ); // Where i is true or false
 
  // tft.fillScreen(TFT_BLACK);
  // tft.drawRect(0, 0, tft.width(), tft.height(), TFT_GREEN);

  // tft.setCursor(0, 4, 4);

  // tft.setTextColor(TFT_WHITE);
  // tft.println(" Invert OFF\n");

  // tft.println(" White text");
  
  // tft.setTextColor(TFT_RED);
  // tft.println(" Red text");
  
  // tft.setTextColor(TFT_GREEN);
  // tft.println(" Green text");
  
  // tft.setTextColor(TFT_BLUE);
  // tft.println(" Blue text");

  // delay(5000);


  // // Binary inversion of colours
  // tft.invertDisplay( true ); // Where i is true or false
 
  // tft.fillScreen(TFT_BLACK);
  // tft.drawRect(0, 0, tft.width(), tft.height(), TFT_GREEN);

  // tft.setCursor(0, 4, 4);

  // tft.setTextColor(TFT_WHITE);
  // tft.println(" Invert ON\n");

  // tft.println(" White text");
  
  // tft.setTextColor(TFT_RED);
  // tft.println(" Red text");
  
  // tft.setTextColor(TFT_GREEN);
  // tft.println(" Green text");
  
  // tft.setTextColor(TFT_BLUE);
  // tft.println(" Blue text");

  // delay(5000);
}