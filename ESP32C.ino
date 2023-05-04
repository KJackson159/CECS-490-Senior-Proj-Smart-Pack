/*
 -This is only the testing code for the 775 12-volt DC motors to check for functionality. After uploading code, please use the external 12v power supply to power the motor driver,
  and use the 5v output from the motor driver to power the ESP32!
 -Sources: https://community.home-assistant.io/t/esp32-cam-fails-when-led-or-pwm-enabled/292872/4 (QnA about PWM on ESP32-CAM)
           https://www.youtube.com/watch?v=qbVXzLIfuVk (using a potentiometer for 775 dc motor)
           https://www.youtube.com/watch?v=_o7hCxYKLyc (Another example using a potentiometer, but includes motor states like forwards and backwards)
           https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/ (borrowed some of code)
           https://randomnerdtutorials.com/esp32-cam-car-robot-web-server/ (other example code)
  -FIRST! Test the motors WITHOUT the potentiometer (so they will move at full speed when operated). THIS IS THE CODE FOR WITHOUT POTENTIOMETER.
  -Next, figure out how to create a similar test with the potentiometer implemented this time.
*/
/*
  ____________________
  |   (FR)   |  (BR) |
  <=         |       |
  |   (FL)   |  (BL) |
  --------------------  
*/
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#include <String.h>

uint8_t broadcastAddress[] = {0x94, 0x3C, 0xC6, 0x32, 0xDD, 0x64};  //UPDATE THIS WITH ADDRESS OF "MCU A"

// Front-Left (FL) Motor A
#define MOTOR_A_EN   12 //GPIO 12: ENA
#define MOTOR_A_PIN1 14 //GPIO 14: IN1
#define MOTOR_A_PIN2 27 //GPIO 27: IN2
// Front-Right (FR) Motor B
#define MOTOR_B_PIN1 26 //GPIO 26: IN3 
#define MOTOR_B_PIN2 25 //GPIO 25: IN4
#define MOTOR_B_EN   33 //GPIO 33: ENB
// Back-Right (BR) Motor C
#define MOTOR_C_EN   23 //GPIO 23: ENA
#define MOTOR_C_PIN1 22 //GPIO 22: IN1
#define MOTOR_C_PIN2 21 //GPIO 21: IN2
// Back-Left (BL) Motor D
#define MOTOR_D_PIN1 19 //GPIO 19: IN3 
#define MOTOR_D_PIN2 18 //GPIO 18: IN4
#define MOTOR_D_EN   4 //GPIO 4: ENB

// Setting PWM properties
#define FREQ 90000
#define PWM_CHL 0 //channel for left wheels
#define PWM_CHR 2 //channel for right wheels
#define RES   8
int dutyL = 255; // x / 255 = duty cycle %
int dutyR = dutyL;
char* motorState;
int modeON = LOW;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    char myState[32]; //forward, reverse, left, right, stop, and exit
    int running; //If HIGH, it's in controller mode. Else, do not move motors at all.
    int faceDetected; //flag used if successful facial recognition
    int unlockRequest; //flag used if user presses unlock
} struct_message;

// Create a struct_message called receive_data
struct_message receive_data;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receive_data, incomingData, sizeof(receive_data));
  Serial.print("Data Received: \n ");
  motorState = receive_data.myState;
  modeON = receive_data.running;
  Serial.print("Motor State: "); Serial.println(motorState);
  Serial.print("Running Mode: "); Serial.println(modeON);
  Serial.println( );
}

void setup() {
  // sets the pins as outputs:
  //Motor A setup
  pinMode(MOTOR_A_PIN1, OUTPUT);
  pinMode(MOTOR_A_PIN2, OUTPUT);
  pinMode(MOTOR_A_EN, OUTPUT);
  //Motor B setup
  pinMode(MOTOR_B_PIN1, OUTPUT);
  pinMode(MOTOR_B_PIN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  //Motor C setup
  pinMode(MOTOR_C_PIN1, OUTPUT);
  pinMode(MOTOR_C_PIN2, OUTPUT);
  pinMode(MOTOR_C_EN, OUTPUT);
  //Motor D setup
  pinMode(MOTOR_D_PIN1, OUTPUT);
  pinMode(MOTOR_D_PIN2, OUTPUT);
  pinMode(MOTOR_D_EN, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(PWM_CHL, FREQ, RES);
  ledcSetup(PWM_CHR, FREQ, RES);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR_A_EN, PWM_CHL);
  ledcAttachPin(MOTOR_B_EN, PWM_CHR);
  ledcAttachPin(MOTOR_C_EN, PWM_CHR);
  ledcAttachPin(MOTOR_D_EN, PWM_CHL);

  motorState = "stop"; //initialize state to STOP
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("Testing DC Motors.");
}

void loop() {
  while(modeON){
    ledcWrite(PWM_CHL, dutyL);
    ledcWrite(PWM_CHR, dutyR);
    // Forward
    if(motorState == "forward"){
      Serial.println("Forward: ");
      dutyL = dutyR = 255;
      digitalWrite(MOTOR_A_PIN1, HIGH);
      digitalWrite(MOTOR_A_PIN2, LOW); 
      digitalWrite(MOTOR_B_PIN1, HIGH);
      digitalWrite(MOTOR_B_PIN2, LOW); 
      digitalWrite(MOTOR_C_PIN1, HIGH);
      digitalWrite(MOTOR_C_PIN2, LOW); 
      digitalWrite(MOTOR_D_PIN1, HIGH);
      digitalWrite(MOTOR_D_PIN2, LOW); 
    }
    // Reverse
    if(motorState == "reverse"){
      Serial.println("Reverse: ");
      dutyL = dutyR = 255;
      digitalWrite(MOTOR_A_PIN1, LOW);
      digitalWrite(MOTOR_A_PIN2, HIGH); 
      digitalWrite(MOTOR_B_PIN1, LOW);
      digitalWrite(MOTOR_B_PIN2, HIGH); 
      digitalWrite(MOTOR_C_PIN1, LOW);
      digitalWrite(MOTOR_C_PIN2, HIGH); 
      digitalWrite(MOTOR_D_PIN1, LOW);
      digitalWrite(MOTOR_D_PIN2, HIGH);
    }
    // Right
    if(motorState == "right"){
      Serial.print("Right ");
      if(MOTOR_A_PIN1 < MOTOR_A_PIN2) Serial.println("(reverse): ");
      else Serial.println("(forward): ");
      dutyL = 255; 
      dutyR = 135; //slow down right wheels
      //If the motor pins are ALL ZERO, the robot will turn right in forward direction by default
      if(MOTOR_A_PIN1 == MOTOR_A_PIN2){
        digitalWrite(MOTOR_A_PIN1, HIGH);
        digitalWrite(MOTOR_A_PIN2, LOW); 
        digitalWrite(MOTOR_B_PIN1, HIGH);
        digitalWrite(MOTOR_B_PIN2, LOW); 
        digitalWrite(MOTOR_C_PIN1, HIGH);
        digitalWrite(MOTOR_C_PIN2, LOW); 
        digitalWrite(MOTOR_D_PIN1, HIGH);
        digitalWrite(MOTOR_D_PIN2, LOW);        
      }
    } 
    // Left
    if(motorState == "left"){
      Serial.println("Left "); 
      if(MOTOR_A_PIN1 < MOTOR_A_PIN2) Serial.println("(reverse): ");
      else Serial.println("(forward): ");
      dutyL = 135; //slow down left wheels
      dutyR = 255;
      //If the motor pins are ALL ZERO, the robot will turn left in forward direction by default
      if(MOTOR_A_PIN1 == MOTOR_A_PIN2){
        digitalWrite(MOTOR_A_PIN1, HIGH);
        digitalWrite(MOTOR_A_PIN2, LOW); 
        digitalWrite(MOTOR_B_PIN1, HIGH);
        digitalWrite(MOTOR_B_PIN2, LOW); 
        digitalWrite(MOTOR_C_PIN1, HIGH);
        digitalWrite(MOTOR_C_PIN2, LOW); 
        digitalWrite(MOTOR_D_PIN1, HIGH);
        digitalWrite(MOTOR_D_PIN2, LOW);        
      }
    } 
    // Stop
    if(motorState == "stop"){
      Serial.println("Stop ");
      if(MOTOR_A_PIN1 < MOTOR_A_PIN2) Serial.println("(reverse): ");
      else Serial.println("(forward): ");
      dutyL = 0; //both dutycycles at 0 (to prevent changing pins' directions)
      dutyR = 0;
    }
    // Exit (Return to main menu)
    if(motorState == "exit"){
      digitalWrite(MOTOR_A_PIN1, LOW);
      digitalWrite(MOTOR_A_PIN2, LOW);
      digitalWrite(MOTOR_B_PIN1, LOW);
      digitalWrite(MOTOR_B_PIN2, LOW);
      digitalWrite(MOTOR_C_PIN1, LOW);
      digitalWrite(MOTOR_C_PIN2, LOW);
      digitalWrite(MOTOR_D_PIN1, LOW);  
      digitalWrite(MOTOR_D_PIN2, LOW);
      //reset state to STOP for next time controller mode is selected
      motorState = "stop";
      modeON = LOW;
      //return to main menu
    }
    delay(100);
  }
}