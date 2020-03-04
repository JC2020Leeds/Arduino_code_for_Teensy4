#include <PWMServo.h>

PWMServo linser_L;
PWMServo linser_R;

int but_pin_linser_L = 3;
int but_pin_linser_R = 4;

int lower_angle = 18;
int upper_angle = 168;
int linser_L_pos = lower_angle;     //linear servo lower responsive angle as 18 deg and upper as 168 deg
int linser_R_pos = upper_angle;

int linser_L_pin = 0;
int linser_R_pin = 1;

int but_L_state = 0;
int but_R_state = 0;


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    pinMode(but_pin_linser_L, INPUT);
    pinMode(but_pin_linser_R, INPUT);
   
    linser_L.attach(linser_L_pin, 544, 2400);  // attached pin number on Teensy as defined above
    linser_L.write(linser_L_pos);
    linser_R.attach(linser_R_pin, 544, 2400);  // attached pin number on Teensy as defined above
    linser_R.write(linser_R_pos);

    Serial.printf("linear servo Left Angle Read %d\r\n", linser_L_pos);
    Serial.printf("Linear servo Right Angle Read %d\r\n\n", linser_R_pos);
    delay(100);
}

void loop() {

  but_L_state = digitalRead(but_pin_linser_L);
  but_R_state = digitalRead(but_pin_linser_R);

  linser_L_pos = linser_L.read();
  linser_R_pos = linser_R.read();
  
  //To extend Left shaft and Right shaft to shut the mechanism
  if (but_L_state == HIGH && linser_L_pos <= lower_angle){
    linser_L.write(upper_angle);
    delay(300);    
  }
  else if (but_L_state == HIGH && linser_L_pos >= upper_angle){
    linser_L.write(lower_angle);
    delay(300);    
  }

  if (but_R_state == HIGH && linser_R_pos >= upper_angle){
    linser_R.write(lower_angle);
    delay(300);      
  }
  else if (but_R_state == HIGH && linser_R_pos <= lower_angle){
    linser_L.write(upper_angle);
    delay(300);    
  }
  Serial.printf("Linear servo Left Angle Read: %d\r\n", linser_L_pos);
  //Serial.printf("Potentiometer control linear servo L: %d\r\n\n", pot_valmap_linser_L);
  Serial.printf("Button linear servo L: %d\r\n\n", but_L_state);
  Serial.printf("Linear servo Right Angle Read: %d\r\n", linser_R_pos);
  //Serial.printf("Potentiometer control linear servo R: %d\r\n\n", pot_valmap_linser_R);
  Serial.printf("Button linear servo R: %d\r\n\n", but_R_state);
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
