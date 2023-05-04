int trigPin = 23;    // TRIG pin
int echoPin = 22;    // ECHO pin
int trigPin2 = 21;  // TRIG pin for sensor 2
int echoPin2 = 5;  //ECHO pin for sensor 2

int leftPin = 14;    // Left motor pin
int rightPin = 12;   // Right motor pin

float duration_us, distance_cm, duration_us2, distance_cm2;

bool flagLeft = false;   // Flag for turning left
bool flagRight = false;  // Flag for turning right

void setup() {
  // begin serial port
  Serial.begin (9600);

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
