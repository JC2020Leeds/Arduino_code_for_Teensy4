#include <PWMServo.h>

PWMServo P16;  // create servo object for controlling servo
                // max 8 servos can be created


int knob_pin = A0; //pin for potentiometer    A0 pin
int pos = 0;    // variable to store servo position
int P16_pin = 9;  // servo pin for FS6530M feetech servo motor
int linearValue_knob = 0;
int knobValue;
//Max/min pulse values in microseconds for the linear actuators

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    P16.attach(P16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    P16.write(0);
    Serial.printf("Angle Read %d\r\n", P16.read());
    delay(100);
}

void loop() {

  knobValue = analogRead(knob_pin);
  linearValue_knob = map(knobValue, 0, 1023, 0, 180);  //Map analog value from the sensor to the linear actuator
  //linearKnob.write(linearValue_knob);
  Serial.printf("Angle Read %d\r\n", P16.read());
  Serial.printf("knob %d\r\n", linearValue_knob);
  P16.write(linearValue_knob);
  delay(20);
  
  
  



  
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    P16.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(30);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    P16.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(30);                       // waits 15ms for the servo to reach the position
//  }
}
