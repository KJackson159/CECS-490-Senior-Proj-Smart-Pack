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

// Motor A
#define MOTOR_A_PIN1 14 //GPIO 14: IN1
#define MOTOR_A_PIN2 27 //GPIO 27: IN2
#define MOTOR_A_EN   12 //GPIO 12: ENA
// Motor B
#define MOTOR_B_PIN1 26 //GPIO 26: IN3 
#define MOTOR_B_PIN2 25 //GPIO 25: IN4
#define MOTOR_B_EN   33 //GPIO 33: ENB
// Setting PWM properties
#define FREQ 1000
#define PWM_CH 0
#define RES   10
int dutyCycle = 1023;

void setup() {
  // sets the pins as outputs:
  //Motor A setup
  pinMode(MOTOR_A_PIN1, OUTPUT);
  pinMode(MOTOR_A_PIN2, OUTPUT);
  pinMode(MOTOR_A_EN, OUTPUT);
  //analogWrite(enable1Pin, 0); //To enable motor A's speed

  //Motor B setup
  pinMode(MOTOR_B_PIN1, OUTPUT);
  pinMode(MOTOR_B_PIN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  //analogWrite(enable2Pin, 0); //To enable motor B's speed
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR_A_EN, PWM_CH);
  ledcAttachPin(MOTOR_B_EN, PWM_CH);
  // configure LED PWM functionalitites
  ledcSetup(PWM_CH, FREQ, RES);

}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  analogWrite(MOTOR_A_EN, dutyCycle);
  analogWrite(MOTOR_B_EN, dutyCycle);
  */
  ledcWrite(PWM_CH, dutyCycle);
  
  // Move the DC motor forward at maximum speed
  digitalWrite(MOTOR_A_PIN1, HIGH);
  digitalWrite(MOTOR_A_PIN2, LOW); 
  digitalWrite(MOTOR_B_PIN1, HIGH);
  digitalWrite(MOTOR_B_PIN2, LOW);   
  delay(2000);

  //Stop
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, LOW);
  delay(1000);
    
  // Move DC motor backwards at maximum speed
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, HIGH); 
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, HIGH); 
  delay(2000);

  // Stop the DC motor
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, LOW);
  delay(1000);
}

