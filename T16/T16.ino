#include <PWMServo.h>

PWMServo myservo;  // create servo object for controlling servo
                // max 8 servos can be created

int pos = 0;    // variable to store servo position
int T16_pin = 10;  // servo pin for FS6530M feetech servo motor

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    myservo.attach(T16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    myservo.write(0);
    Serial.printf("Angle Read %d\r\n", myservo.read());
    delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
for(pos = 0; pos<=180; pos += 1)  //goes from 0 deg to 180 deg (max is 180deg)
{
  myservo.write(pos);
  Serial.printf("Angle Read %d\r\n", myservo.read());

  delay(100);
}
for(pos = 180; pos>=0; pos -= 1)
{
  myservo.write(pos);
  Serial.printf("Angle Read %d\r\n", myservo.read());

  delay(100);
}
}
