/* run from an ESP32 Dev module
  using an electromechanical latch lock
  detects if latch is closed or open
  set to unlock every 10 seconds  
*/
#define SOLENOID 23
#define Lswitch 22
int flag = 0;
int swiValue = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(SOLENOID, OUTPUT);
  pinMode(Lswitch, INPUT);

}

void loop() {
  // */
  swiValue = analogRead(Lswitch);
  delay(100);
  //Serial.println(swiValue);

  digitalWrite(SOLENOID, HIGH);
  //Serial.println("Unlocking Lock");
  delay(100);

  digitalWrite(SOLENOID, LOW);
  //Serial.println("");
    delay(1000);
    // */

  if( (digitalRead(Lswitch) == HIGH) && (flag == 0) )
  {
    Serial.println("unlocked");
    flag = 1;
    delay(10000);
  }

  if( (digitalRead(Lswitch) == LOW) && (flag == 1) )
  {
    Serial.println("locked");
    flag = 0;
    delay(1000);
  } 

}

