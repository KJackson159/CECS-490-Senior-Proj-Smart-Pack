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
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#include <TinyGPSPlus.h>

// Global copy of slave
esp_now_peer_info_t peerInfo;
TinyGPSPlus gps;
#define CHANNEL 1

//MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

float latitude;
float longitude;
int send_faceDetected;
int send_motorControl;
int receive_faceDetected;
int receive_motorControl;
String success;

//define data structure
typedef struct struct_message{
  int b;
  int c;
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

void sendData() {
  //data++;
  const uint8_t *peer_addr = peerInfo.peer_addr;
  //Serial.print("Sending: "); Serial.println(data);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &send_data, sizeof(send_data));

  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
  //Serial.print("Latitude: "); Serial.println(data.a);
  //Serial.print("Longitude: "); Serial.println(data.b);
}

// callback when data is sent from Master to receiver
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}
  //char macStr[18];
  //snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  //Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  //Serial.println(data);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow Setup Running");
  InitESPNow();
  // Once ESPNow is successfully Init,register for Send CB of Trasnmitted packet
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
  //Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv); 
}

// callback when data is recv 
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  memcpy(&receive_data, incomingData, sizeof(receive_data));
  Serial.println("Data Received:");
  receive_faceDetected = receive_data.b;
  receive_motorControl = receive_data.c;
  Serial.print("Face Detected "); Serial.println(receive_faceDetected);
  Serial.print("Motor Controls: "); Serial.println(receive_motorControl);
  Serial.println( );
  //char macStr[18];
  //snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           //mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print("Last Packet Recv from: "); Serial.println(macStr);
}

void loop() {
  
      send_faceDetected = 1;  //TEST
      send_motorControl = 3; // TEST
      send_data.b = send_faceDetected;
      send_data.c = send_motorControl;
      sendData(); 
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

  // wait for 3seconds to run the logic again
  delay(3000);
}
