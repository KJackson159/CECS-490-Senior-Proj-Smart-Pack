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

#include <Arduino.h>
#include "HX711.h"
#include "soc/rtc.h"
#include <Wire.h>
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h> // For BME280
#include <Pushbutton.h>
#include <SPI.h>

// For the breakout board, you can use any 2 or 3 pins.
// These pins will also work for the 1.8" TFT shield. 
#define TFT_SCLK      18        // SPI clock
#define TFT_MOSI      23         // SPI Data
#define TFT_CS        5
#define TFT_RST        19 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         2 

// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;
int weight;
int lastWeight;
//REPLACE WITH YOUR CALIBRATION FACTOR
#define CALIBRATION_FACTOR 411

// BME280 Reference
int temperature;
int lastTemperature;
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
unsigned long delayTime;

//Button
#define BUTTON_PIN 19
Pushbutton button(BUTTON_PIN);

void setup() {
  Serial.begin(115200);

  // OR use this initializer if using a 1.8" TFT screen with offset such as WaveShare:
  tft.initR(INITR_GREENTAB);      // Init ST7735S chip, green tab
  
  // HX711
  setCpuFrequencyMhz(RTC_CPU_FREQ_80M);
  
  Serial.println("Initializing the scale");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  scale.set_scale(CALIBRATION_FACTOR);   // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();               // reset the scale to 0
  
  Serial.println(F("BME280 test"));

  // BME280
  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();

  // Black display
  tft.fillScreen(ST77XX_BLACK);
}

void loop() {
  printValues();
  delay(delayTime);

  if (scale.wait_ready_timeout(200)) {
    weight = round(scale.get_units());
    temperature = (1.8 * bme.readTemperature() + 32);
    Serial.print("Weight: ");
    Serial.println(weight);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    if ((weight != lastWeight) || (temperature != lastTemperature)){
      // HX711
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 0);
      tft.println("Weight: ");
      tft.setCursor(20, 10);
      tft.print(weight / 453.592);
      tft.setCursor(40, 10);
      tft.print(" lbs");
      // BME280
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

void printValues() {
  // Convert temperature to Celsius
  /*Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");*/
  
  // Convert temperature to Fahrenheit
  Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}
