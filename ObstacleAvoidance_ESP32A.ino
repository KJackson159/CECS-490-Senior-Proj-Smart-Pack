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

int leftPin = 14;    // Left motor pin
int rightPin = 12;   // Right motor pin

float duration_us, distance_cm, duration_us2, distance_cm2;

bool flagLeft = false;   // Flag for turning left
bool flagRight = false;  // Flag for turning right

//MAC Address of your receiver 
uint8_t broadcastAddressB[] = {0xCC, 0xDB, 0xA7, 0x51, 0x06, 0x60}; //ESP32_B  CC:DB:A7:51:06:60
uint8_t broadcastAddressC[] = {0xEC, 0x62, 0x60, 0x1C, 0x89, 0x30}; //ESP32_C EC:62:60:1C:89:30

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
  
  // configure the motor pins to output mode
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
}

void loop() {
  if(receive_faceDetected == 1){
      send_data.faceDetected = send_faceDetected;
      esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  }


  if(receive_motorControl == 1 ){
      send_data.running = send_motorControl;
      esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  }
  if(receive_unlockRequest == 1 ){
      send_data.unlockRequest = send_unlockRequest;
      esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));
  }

      //esp_err_t result = esp_now_send(0, (uint8_t *) &send_data, sizeof(send_data));

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

  // wait for 5 seconds to run the logic again
  delay(5000);

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
    
    // set the appropriate flag for turning left or right
    if (distance_cm < distance_cm2) {
      flagLeft = true;
    } else {
      flagRight = true;
    }
  } else {
    Serial.println("");
    
    // reset the flags if no obstacle is detected
    flagLeft = false;
    flagRight = false;
  }

  // turn left or right based on the flags
  if (flagLeft) {
    Serial.println("Turning left");
    digitalWrite(leftPin, HIGH);
    digitalWrite(rightPin, LOW);
  } else if (flagRight) {
    Serial.println("Turning right");
    digitalWrite(leftPin, LOW);
    digitalWrite(rightPin, HIGH);
  } else {
    Serial.println("Moving forward");
    digitalWrite(leftPin, HIGH);
    digitalWrite(rightPin, HIGH);
  }
  
  delay(200);
}

