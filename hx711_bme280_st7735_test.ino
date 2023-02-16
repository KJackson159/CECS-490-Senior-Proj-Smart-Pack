/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-load-cell-hx711/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// Library HX711 by Bogdan Necula: https://github.com/bogde/HX711
// Library: pushbutton by polulu: https://github.com/pololu/pushbutton-arduino

/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

// Include libraries:
#include "HX711.h" // Hardware-specific library for HX711
#include <Wire.h> // Allows use of I2C for BME280
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_BME280.h> // Hardware-specific library for BME280
#include <Adafruit_Sensor.h> // For BME280
#include <SPI.h>

// ESP32 GPIO pins used by ST7735:
#define TFT_SCLK      18  // SPI clock
#define TFT_MOSI      23  // SPI Data
#define TFT_CS         5
#define TFT_RST       19  // Reset
#define TFT_DC         2 

// ESP32 GPIO pins used by HX711:
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

// TFT graphics library usage for 1.8" or 1.44" ST7735:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// I2C communications:
HX711 scale; // HX711 library instance for future reference
Adafruit_BME280 bme; // BME280 library instance for future reference

// Value calculated from calibration of HX711 with known weights:
#define CALIBRATION_FACTOR 411

// Initialize variables
int weight;
int lastWeight;
int temperature;
int lastTemperature;


void setup() {
  Serial.begin(115200); // Initialize Serial monitor

  // OR use this initializer if using a 1.8" TFT screen with offset such as WaveShare:
  tft.initR(INITR_GREENTAB);  // Init ST7735S chip, green tab

  // Initialize load cell with begin() method on the scale object; uses GPIO pins as arguments:
  Serial.println("Initializing the scale...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Further initializing of scale:
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare(); // Resets scale to 0

  // BME280
  bool status;

  // Initialization/detection of BME280:
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("BME280 not found.");
    while (1);
  }

  // Black display
  tft.fillScreen(ST77XX_BLACK);
}

void loop() {
  // Non-blocking method to get readings; maximum timeout to wait for HX711 to be initialized/detected:
  if (scale.wait_ready_timeout(200)) {
    weight = round(scale.get_units()); // Gets one single scale reading
    temperature = (1.8 * bme.readTemperature() + 32); // Temperature converted to Fahrenheit
    
    // Serial Monitor printing weight and temperature values:
    Serial.println("Weight: ");
    Serial.println(weight);
    Serial.print(" lbs");
    Serial.println("Temperature: ");
    Serial.println(temperature);
    Serial.print(" *F");

    // ST7735 text color and size:
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);

    // Update weight and temperature in real-time:
    if ((weight != lastWeight) || (temperature != lastTemperature)){
      tft.fillScreen(ST77XX_BLACK); // Allows screen to refresh with new values
      
      // Displays weight value on ST7735:
      tft.setCursor(0, 0);
      tft.println("Weight: ");
      tft.setCursor(20, 10);
      tft.print(weight / 453.592);
      tft.setCursor(40, 10);
      tft.print(" lbs");

      // Displays temperature value on ST7735:
      tft.setCursor(0, 30);
      tft.println("Temperature: ");
      tft.setCursor(20, 40);
      tft.print(1.8 * bme.readTemperature() + 32);
      tft.print(" *F");
    }
    lastTemperature = temperature;
    lastWeight = weight;
  }
  else {
    Serial.println("HX711 not found.");
  }
}
