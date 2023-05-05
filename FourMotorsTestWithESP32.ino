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
#define PWM_CHF 0 //channel for front wheels
#define PWM_CHR 1 //channel for back right wheel
#define PWM_CHL 2 //channel for back left wheel
#define RES   8
int initduty = 255; // x / 255 = duty cycle %

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
  ledcSetup(PWM_CHF, FREQ, RES);
  ledcSetup(PWM_CHR, FREQ, RES);
  ledcSetup(PWM_CHL, FREQ, RES);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR_A_EN, PWM_CHF);
  ledcAttachPin(MOTOR_B_EN, PWM_CHF);
  ledcAttachPin(MOTOR_C_EN, PWM_CHR);
  ledcAttachPin(MOTOR_D_EN, PWM_CHL);
  
  Serial.begin(115200);
  Serial.println("Testing DC Motors.");

}

void loop() {
  // Move the DC motor forward at maximum speed
  ledcWrite(PWM_CHF, initduty);
  ledcWrite(PWM_CHR, initduty);
  ledcWrite(PWM_CHL, initduty);
  Serial.print("Forward: ");
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW); 
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW); 
  
  digitalWrite(MOTOR_C_PIN1, HIGH);
  digitalWrite(MOTOR_C_PIN2, LOW); 
  digitalWrite(MOTOR_D_PIN1, HIGH);
  digitalWrite(MOTOR_D_PIN2, LOW); 
  delay(2500);

  //Stop
  Serial.print("pause1: ");
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);  
  digitalWrite(MOTOR_B_PIN2, LOW);

  digitalWrite(MOTOR_C_PIN1, LOW);
  digitalWrite(MOTOR_C_PIN2, LOW);
  digitalWrite(MOTOR_D_PIN1, LOW);  
  digitalWrite(MOTOR_D_PIN2, LOW);
  delay(1000);
    
  // Move DC motor backwards at maximum speed
  Serial.print("Backward: ");
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, HIGH); 
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, HIGH); 
  
  digitalWrite(MOTOR_C_PIN1, LOW);
  digitalWrite(MOTOR_C_PIN2, HIGH); 
  digitalWrite(MOTOR_D_PIN1, LOW);
  digitalWrite(MOTOR_D_PIN2, HIGH); 
  delay(2500);
  
  // Stop the DC motor
  Serial.print("pause2: ");
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, LOW);

  digitalWrite(MOTOR_C_PIN1, LOW);
  digitalWrite(MOTOR_C_PIN2, LOW);
  digitalWrite(MOTOR_D_PIN1, LOW);  
  digitalWrite(MOTOR_D_PIN2, LOW);
  delay(1000);

  Serial.print("CW: ");
  ledcWrite(PWM_CHR, 150);
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW); 
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW); 
  
  digitalWrite(MOTOR_C_PIN1, HIGH);
  digitalWrite(MOTOR_C_PIN2, LOW); 
  digitalWrite(MOTOR_D_PIN1, HIGH);
  digitalWrite(MOTOR_D_PIN2, LOW); 
  delay(2500);

  //Stop
  Serial.print("pause3: ");
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);  
  digitalWrite(MOTOR_B_PIN2, LOW);

  digitalWrite(MOTOR_C_PIN1, LOW);
  digitalWrite(MOTOR_C_PIN2, LOW);
  digitalWrite(MOTOR_D_PIN1, LOW);  
  digitalWrite(MOTOR_D_PIN2, LOW);
  delay(1000);

  Serial.print("CCW: ");
  ledcWrite(PWM_CHL, 150);
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW); 
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW); 
  
  digitalWrite(MOTOR_C_PIN1, HIGH);
  digitalWrite(MOTOR_C_PIN2, LOW); 
  digitalWrite(MOTOR_D_PIN1, HIGH);
  digitalWrite(MOTOR_D_PIN2, LOW); 
  delay(2500);

  //Stop
  Serial.print("pause4: ");
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);  
  digitalWrite(MOTOR_B_PIN2, LOW);

  digitalWrite(MOTOR_C_PIN1, LOW);
  digitalWrite(MOTOR_C_PIN2, LOW);
  digitalWrite(MOTOR_D_PIN1, LOW);  
  digitalWrite(MOTOR_D_PIN2, LOW);
  delay(1000);
}

