#include <PWMServo.h>

PWMServo myservo;  // create servo object for controlling servo
                // max 8 servos can be created

int pos = 0;    // variable to store servo position
int FS6530M_pin = 2;  // servo pin for FS6530M feetech servo motor

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    myservo.attach(FS6530M_pin, 544, 2400);  // attached pin number on Teensy as defined above
    myservo.write(0);
    Serial.printf("Angle Read %d\r\n", myservo.read());
    delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(200);                       // wait for a 
for(pos = 10; pos<180; pos += 1)  //goes from 10 deg to 110 deg (max is 120deg)
{
  myservo.write(pos);
  
  delay(15);
}
  Serial.printf("Angle Read %d\r\n", myservo.read());
for(pos = 180; pos>=1; pos -= 1)
{
  myservo.write(pos);
  delay(15);
}
}
