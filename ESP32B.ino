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
#include <esp_now.h>
#include <WiFi.h>

// ESP32 GPIO pins used by ST7735:
#define TFT_SCLK      18  // SPI clock
#define TFT_MOSI      23  // SPI Data
#define TFT_CS         5
#define TFT_RST       19  // Reset
#define TFT_DC         2 
#define CALIBRATION_FACTOR -449 // Value calculated from calibration of HX711 with known weights
esp_now_peer_info_t peerInfo;
#define CHANNEL 0         //ESPNOW

// ESP32 GPIO pins used by latch lock:
#define SOLENOID 15
#define Lswitch 17

// ESP32 GPIO pins used by HX711:
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

// TFT graphics library usage for 1.8" or 1.44" ST7735:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// I2C communications:
HX711 scale; // HX711 library instance for future reference
Adafruit_BME280 bme; // BME280 library instance for future reference

//MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x94, 0x3C, 0xC6, 0x32, 0xDD, 0x64};  //ESP32_A MAC Address

// Initialize variables
int weight;
int lastWeight;
int temperature;
int lastTemperature;
int lockStatus; // Flag communication variable with server
int exitFlag;
int faceDetectFlag;
int send_faceDetected;
int receive_faceDetected;
int receive_unlockRequest = 0;
String success;

//define data structure
typedef struct struct_message{
  char myState[32]; //forward, reverse, left, right, stop, and exit
  int running; //If HIGH, it's in controller mode. Else, do not move motors at all.
  int faceDetected; //flag used if successful facial recognition
  int unlockRequest; //flag used if user presses unlock
} struct_message;

//create structered data object
struct_message send_data;
struct_message receive_data;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// callback when data is sent from Master to receiver
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
}

// callback when data is recv
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  memcpy(&receive_data, incomingData, sizeof(receive_data));
  Serial.println("Recv Data: ");
  receive_faceDetected = receive_data.faceDetected;
  receive_unlockRequest = receive_data.unlockRequest;
  Serial.print("Face Detected: "); Serial.println(receive_faceDetected);
  Serial.print("Unlock Request: "); Serial.println(receive_unlockRequest);
  Serial.println( );
  char macStr[18];  //used to print out MAC address from transmitting ESP32
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);

}

void setup() {
  Serial.begin(115200); // Initialize Serial monitor
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow Setup Running");
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info.
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Once ESPNow is successfully Init,register for Send CB of Trasnmitted packet
  esp_now_register_recv_cb(OnDataRecv);

  // OR use this initializer if using a 1.8" TFT screen with offset such as WaveShare:
  tft.initR(INITR_GREENTAB);  // Init ST7735S chip, green tab

  // Initialize load cell with begin() method on the scale object; uses GPIO pins as arguments:
  Serial.println("Initializing the scale...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Latch lock setup:
  pinMode(SOLENOID, OUTPUT); //pin that controls signal to unlock latch
  pinMode(Lswitch, INPUT);
  digitalWrite(SOLENOID, LOW);  // initialize pin LOW

  // Further initializing of scale:
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare(); // Resets scale to 0

  bool status;   // BME280

  // Initialization/detection of BME280:
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("BME280 not found.");
    while (1);
  }
  tft.fillScreen(ST77XX_BLACK);   // Black display
  tft.setTextColor(ST77XX_WHITE);  // ST7735 text color and size:
  tft.setTextSize(1);
  // Latch lock setup:
  lockStatus = analogRead(Lswitch);  //changed from swiValue(variable) =
  delay(100);
  //Serial.println(swiValue);
}

void loop() {
  send_faceDetected = 2; //TEST
  send_data.faceDetected = send_faceDetected;
  //esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));

  if ((digitalRead(Lswitch) == HIGH) && (lockStatus == 0)) {
    Serial.println("unlocked");
    lockStatus = 1;
    delay(5000);
  }
  if ((digitalRead(Lswitch) == LOW) && (lockStatus == 1)) {
    Serial.println("locked");
    lockStatus = 0;
    delay(1000);
  } 

  //receive_faceDetected = 1; // TEST
  // If an enrolled face is detected:
  if (receive_faceDetected == 1) {
    faceDetectFlag = 1;
    lockStatus = 1;
    tft.setCursor(0, 80);
    tft.setTextSize(2);
    tft.print("Hello,");
    tft.setCursor(0, 100);
		tft.print("[Name]!"); // LCD displays user's saved name
    
	  // Loop while user is in Menu:
    while (faceDetectFlag == 1) {
	
      // If user presses unlock button in Menu:
      if (lockStatus == 0) {
        digitalWrite(SOLENOID, HIGH);
        delay(500);
		  
		    // Display Smart Lock unlocked on LCD:
        tft.fillScreen(ST77XX_BLACK); // Allows screen to refresh 
        tft.setCursor(0, 80);
        tft.setTextSize(2);
        tft.print("Smart Lock");
        tft.setCursor(10, 100);
		    tft.print("Unlocked!");
		    delay(3000); // Display unlock text for 3 seconds
		    lockStatus = 1;
      }

      delay(5000);
      exitFlag = 1;
	    // Exit Menu when user presses Exit button:
	    if (exitFlag == 1) {
	      faceDetectFlag = 0;
        receive_faceDetected = 0;
        tft.fillScreen(ST77XX_BLACK); // Allows screen to refresh
	    }
	  }
  }

  // DEFAULT LCD DISPLAY MODE (Weight, temperature, and lock status display):
  // Non-blocking method to get readings; maximum timeout to wait for HX711 to be initialized/detected:
    if (scale.wait_ready_timeout(200)) {
    weight = round(scale.get_units()); // Gets one single scale reading
    temperature = (1.8 * bme.readTemperature() + 32); // Temperature converted to Fahrenheit
    
    // Serial Monitor printing weight and temperature values, as well as lock status:
    Serial.println("\n\nWeight: ");
    Serial.print(weight);
    Serial.print(" lbs");
    Serial.println("\n\nTemperature: ");
    Serial.print(1.8 * bme.readTemperature() + 32);
    Serial.print(" *F");
    Serial.println("\n\nLock Status: ");
    // If Smart Pack is unlocked:
    if (lockStatus == 0) {
      Serial.print("Unlocked.");
    }
    // If Smart Pack is locked:
    else {
      Serial.print("Locked.");
    }
    delay(1000);

    // ST7735 text color and size:
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);

    // Update weight and temperature in real-time:
    if ((weight != lastWeight) || (temperature != lastTemperature)){
      delay(1000);
      tft.fillScreen(ST77XX_BLACK); // Allows screen to refresh with new values
      
      // Displays weight value on ST7735:
      tft.setCursor(0, 0);
      tft.println("Weight:");
      tft.setCursor(20, 10);
      tft.print(weight);
      tft.setCursor(40, 10);
      tft.print(" lbs");
      
      // Ounces measurement:
      /*tft.setCursor(20, 20);
      tft.print(weight / 28.35);
      tft.setCursor(40, 20);
      tft.print(" oz");*/

      // Displays temperature value on ST7735 if temperature reaches 120 degrees or higher:
      if (temperature >= 120) {
        tft.setCursor(0, 30);
        tft.println("Temperature:");
        tft.setCursor(20, 40);
        tft.print(1.8 * bme.readTemperature() + 32);
        tft.print(" *F ");
        tft.setTextColor(ST77XX_BLUE);
        tft.print("(CAUTION: TOO HOT. LET REST.)"); // Caution message for user that temp. is too hot

        // Displays smart lock status on ST7735:
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(0, 70);
        tft.println("Lock Status:");
		    tft.setCursor(20, 80);
		    tft.print("Locked.");
      }
      
      // If temperature is at safe value (<= 120 degrees):
      else {
        tft.setCursor(0, 30);
        tft.println("Temperature:");
        tft.setCursor(20, 40);
        tft.print(1.8 * bme.readTemperature() + 32);
        tft.print(" *F");

        // Displays smart lock status on ST7735:
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(0, 60);
        tft.println("Lock Status:");
		    tft.setCursor(20, 70);
		    tft.print("Locked.");
      }
    }
    lastTemperature = temperature;
    lastWeight = weight;
  }
  else {
    Serial.println("HX711 not found.");
  }
}
