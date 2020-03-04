#include <PWMServo.h>

PWMServo linser;
int lin_servo_pin = 2;
int pos = linser.read();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  linser.attach(lin_servo_pin, 0, 2000);  //pin number, min pulse, max pulse
  linser.write(linser.read());
  Serial.printf("Angle Read %d\r\n", linser.read());
}

void loop() {
  // put your main code here, to run repeatedly:
  while(true)
    {
      Serial.printf("%d\r\n", linser.read());
      delay(30);
    }
}
