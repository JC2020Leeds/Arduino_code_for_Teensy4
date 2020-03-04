#include <PWMServo.h>

PWMServo P16;  // create servo object for controlling servo
PWMServo T16;
                // max 8 servos can be created

int P16pos = 0;    // variable to store servo position
int T16pos = 0;    // variable to store servo position

int P16_pin = 9;  // servo pin for P16 actuonix linear actuator
int T16_pin = 10;  // servo pin for T16 actuonix linear actuator


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    P16.attach(P16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    P16.write(0);
    T16.attach(T16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    //T16.attach(T16_pin);
    T16.write(0);
    Serial.printf("P16 Angle Read %d\r\n", P16.read());
    Serial.printf("T16 Angle Read %d\r\n", T16.read());
    delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
for(P16pos = 0; P16pos<=180; P16pos += 1)  //goes from 0 deg to 180 deg (max is 180deg)
{
  P16.write(P16pos);
  Serial.printf("P16 Angle Read %d\r\n", P16.read());

  delay(50);
}

for(T16pos = 0; T16pos<=180; T16pos += 1)  //goes from 0 deg to 180 deg (max is 180deg)
{
  T16.write(T16pos);
  Serial.printf("T16 Angle Read %d\r\n", T16.read());

  delay(50);
}


for(P16pos = 180; P16pos>=0; P16pos -= 1)
{
  P16.write(P16pos);
  Serial.printf("P16 Angle Read %d\r\n", P16.read());

  delay(50);
}

for(T16pos = 180; T16pos>=0; T16pos -= 1)
{
  T16.write(T16pos);
  Serial.printf("T16 Angle Read %d\r\n", T16.read());

  delay(50);        //delay(50) went full range but not delay(100)
}
}
