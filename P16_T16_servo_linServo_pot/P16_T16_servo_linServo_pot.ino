#include <PWMServo.h>

PWMServo P16;  // create servo object for controlling servo
PWMServo T16;  // max 8 servos can be created
PWMServo FEESER;
PWMServo linser_L;
PWMServo linser_R;


int pot_pin_P16 = A0; //pin for potentiometer    A0 pin
int pot_pin_T16 = A1;
int pot_pin_FEESER = A2;
int pot_pin_linser_L = A3;
int pot_pin_linser_R = A4;

int P16_pos = 0;    // variable to store servo position
int T16_pos = 0;    // variable to store servo position
int FEESER_pos = 0;
int linser_L_pos = 0;
int linser_R_pos = 0;

int P16_pin = 9;  // servo pin for FS6530M feetech servo motor
int T16_pin = 10;  // servo pin for T16 actuonix linear actuator
int FEESER_pin = 2;
int linser_L_pin = 0;
int linser_R_pin = 1;


int pot_val_P16;
int pot_val_T16;
int pot_val_FEESER;
int pot_val_linser_L;
int pot_val_linser_R;

int pot_valmap_P16 = 0;
int pot_valmap_T16 = 0;
int pot_valmap_FEESER = 0;
int pot_valmap_linser_L = 0;
int pot_valmap_linser_R = 0;


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    P16.attach(P16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    P16.write(0);
    T16.attach(T16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    T16.write(0);
    FEESER.attach(FEESER_pin, 544, 2400);  // attached pin number on Teensy as defined above
    FEESER.write(0);
    linser_L.attach(linser_L_pin, 544, 2400);  // attached pin number on Teensy as defined above
    linser_L.write(0);
    linser_R.attach(linser_R_pin, 544, 2400);  // attached pin number on Teensy as defined above
    linser_R.write(0);
    Serial.printf("P16 Angle Read %d\r\n", P16.read());
    Serial.printf("T16 Angle Read %d\r\n", T16.read());
    Serial.printf("FEETECH servo Angle Read %d\r\n", FEESER.read());
    Serial.printf("Linear servo Left Angle Read %d\r\n", linser_L.read());
    Serial.printf("Linear servo Right Angle Read %d\r\n\n", linser_R.read());
    delay(100);
}

void loop() {

  pot_val_P16 = analogRead(pot_pin_P16);
  pot_val_T16 = analogRead(pot_pin_T16);
  pot_val_FEESER = analogRead(pot_pin_FEESER);
  pot_val_linser_L = analogRead(pot_pin_linser_L);
  pot_val_linser_R = analogRead(pot_pin_linser_R);
  
  pot_valmap_P16 = map(pot_val_P16, 0, 1023, 0, 180);  //Map analog value from the sensor to the linear actuator
  pot_valmap_T16 = map(pot_val_T16, 0, 1023, 0, 180);
  pot_valmap_FEESER = map(pot_val_FEESER, 0, 1023, 0, 180);
  pot_valmap_linser_L = map(pot_val_linser_L, 0, 1023, 0, 180);
  pot_valmap_linser_R = map(pot_val_linser_R, 0, 1023, 0, 180);
  
  Serial.printf("Angle Read: %d\r\n", P16.read());
  Serial.printf("Potentiometer control P16: %d\r\n\n", pot_valmap_P16);
  Serial.printf("Angle Read: %d\r\n", T16.read());
  Serial.printf("Potentiometer control T16: %d\r\n\n", pot_valmap_T16);
  Serial.printf("Angle Read: %d\r\n", FEESER.read());
  Serial.printf("Potentiometer control Feetech Servo: %d\r\n\n", pot_valmap_FEESER);
  Serial.printf("Angle Read: %d\r\n", linser_L.read());
  Serial.printf("Potentiometer control linear servo L: %d\r\n\n", pot_valmap_linser_L);
  Serial.printf("Angle Read: %d\r\n", linser_R.read());
  Serial.printf("Potentiometer control linear servo R: %d\r\n\n", pot_valmap_linser_R);
  
  P16.write(pot_valmap_P16);
  T16.write(pot_valmap_T16);
  FEESER.write(pot_valmap_FEESER);
  linser_L.write(pot_valmap_linser_L);
  linser_R.write(pot_valmap_linser_R);

  
  delay(200);
  
  
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
