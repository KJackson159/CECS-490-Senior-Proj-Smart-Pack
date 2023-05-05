/**
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Master module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Master >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/
#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <String.h>

// Global copy of slave
esp_now_peer_info_t peerInfo;
TinyGPSPlus gps;
#define CHANNEL 0

int trigPin = 23;    // TRIG pin
int echoPin = 22;    // ECHO pin
int trigPin2 = 21;  // TRIG pin for sensor 2
int echoPin2 = 5;  //ECHO pin for sensor 2
int tempSwitch = 34; // temp switch for Face detected flag
float duration_us, distance_cm, duration_us2, distance_cm2;

//MAC Address of your receiver 
uint8_t broadcastAddressB[] = {0xEC, 0x62, 0x60, 0x76, 0xBD, 0x18}; //ESP32_B  EC:62:60:76:BD:18  LCD, WEIGHT, TEMP, LOCK
uint8_t broadcastAddressC[] = {0xC8, 0xF0, 0x9E, 0x9F, 0x47, 0x38}; //ESP32_C C8:F0:9E:9F:47:38  MOTOR CONTROLLER 

float latitude;
float longitude;
char send_motorState;
char receive_motorState;
int send_motorControl;
int send_faceDetected;
int receive_motorControl;
int receive_faceDetected;
int receive_unlockRequest;
int send_unlockRequest; 
String success;

//define data structure
typedef struct struct_message{
  char myState[32]; //forward, reverse, left, right, stop, and exit
  int running; //If HIGH, it's in controller mode. Else, do not move motors at all.
  int faceDetected; //flag used if successful facial recognition
  int unlockRequest; //flag used if user presses unlock
} struct_message; 

//create structured data object 
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
  Serial.print("\r\nLast Packet Send Status:  ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    char macStr[18]; //^^goes inside OnDataSent | used to debug
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
}


// callback when data is recv 
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  memcpy(&receive_data, incomingData, sizeof(receive_data));
  Serial.println("Data Received:");
  receive_faceDetected = receive_data.faceDetected;
  Serial.print("Face Detected flag: "); Serial.println(receive_faceDetected);
  Serial.println( );
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  WiFi.mode(WIFI_STA);    //Set device in STA mode to begin with
  Serial.println("ESPNow Setup Running");
  InitESPNow();
  // Once ESPNow is successfully Init,register for Send CB of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Register first peer
  memcpy(peerInfo.peer_addr, broadcastAddressB, 6);       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add ESP32_B");
    return;
  }
  // Register first peer
  memcpy(peerInfo.peer_addr, broadcastAddressC, 6);       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add ESP32_C");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
      // configure the trigger pin to output mode
  pinMode(trigPin, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  
  // configure the echo pin to input mode
  pinMode(echoPin, INPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {
  if(receive_faceDetected == 1){
      send_data.faceDetected = send_faceDetected;
      esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
      receive_faceDetected = 0;    //hard coded to a switch since ESP32CAM having issues 
  }

  //  if(receive_motorState == "forward" ){
  //     strcpy(send_data.myState, "forward");
  //     esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  // }
  // if(receive_motorState == "reverse" ){
  //     strcpy(send_data.myState, "reverse");
  //     esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));    
  // }
  // if(receive_motorState == "left" ){
  //     strcpy(send_data.myState, "left");
  //     esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  // }
  // if(receive_motorState == "right" ){
  //     strcpy(send_data.myState, "right");
  //     esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  // }
  // if(receive_motorState == "stop" ){
  //     strcpy(send_data.myState, "stop");
  //     esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  // }
  // if(receive_motorState == "exit" ){
  //     strcpy(send_data.myState, "exit");
  //     esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  // }
  if(receive_motorControl == 1 ){
      send_data.running = send_motorControl;
      esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  }
  if(receive_unlockRequest == 1 ){
      send_data.unlockRequest = send_unlockRequest;
      esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  }

  if(tempSwitch = HIGH){
      receive_faceDetected = 1;    //hard coded to a switch since ESP32CAM having issues 
  }

    while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      latitude = gps.location.lat();
      longitude = gps.location.lng(); 
      //displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(echoPin, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  // generate 10-microsecond pulse to TRIG pin 2
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  duration_us2 = pulseIn(echoPin2, HIGH);
  distance_cm2 = 0.017 * duration_us2;

  // print the distance values to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.print(" cm, Distance 2: ");
  Serial.print(distance_cm2);
  Serial.print(" cm");

   // check if obstacle is detected
  if (distance_cm < 10 || distance_cm2 < 10) {
    Serial.println(" - OBSTACLE DETECTED!");
      strcpy(send_data.myState, "stop");
      esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  }
  // wait for 5 seconds to run the logic again
  delay(5000);
}
