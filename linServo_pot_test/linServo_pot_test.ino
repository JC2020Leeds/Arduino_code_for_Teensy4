#include <PWMServo.h>

PWMServo linser_L;
PWMServo linser_R;

int pot_pin_linser_L = A3;
int pot_pin_linser_R = A4;

int linser_L_pos = 0;
int linser_R_pos = 0;

int linser_L_pin = 0;
int linser_R_pin = 1;

int pot_val_linser_L;
int pot_val_linser_R;

int pot_valmap_linser_L = 0;
int pot_valmap_linser_R = 0;

int lower_angle = 18;
int upper_angle = 168;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    linser_L.attach(linser_L_pin, 544, 2400);  // attached pin number on Teensy as defined above
    linser_L.write(0);
    linser_R.attach(linser_R_pin, 544, 2400);  // attached pin number on Teensy as defined above
    linser_R.write(0);

    Serial.printf("Linear servo Left Angle Read %d\r\n", linser_L.read());
    Serial.printf("Linear servo Right Angle Read %d\r\n\n", linser_R.read());
    delay(100);
}

void loop() {

  pot_val_linser_L = analogRead(pot_pin_linser_L);
  pot_val_linser_R = analogRead(pot_pin_linser_R);
  
  pot_valmap_linser_L = map(pot_val_linser_L, 0, 1023, 0, 180);
  pot_valmap_linser_R = map(pot_val_linser_R, 0, 1023, 0, 180);
  
  Serial.printf("Angle Read: %d\r\n", linser_L.read());
  Serial.printf("Potentiometer control linear servo L: %d\r\n\n", pot_valmap_linser_L);
  Serial.printf("Angle Read: %d\r\n", linser_R.read());
  Serial.printf("Potentiometer control linear servo R: %d\r\n\n", pot_valmap_linser_R);
  
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
