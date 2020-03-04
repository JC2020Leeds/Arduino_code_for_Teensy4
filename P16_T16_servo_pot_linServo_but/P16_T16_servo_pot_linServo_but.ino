#include <PWMServo.h>

PWMServo P16;  // create servo object for controlling servo
PWMServo T16;  // max 8 servos can be created
PWMServo FEESER;
PWMServo linser_L;
PWMServo linser_R;


int pot_pin_P16 = A0; //pin for potentiometer    A0 pin
int pot_pin_T16 = A1;
int pot_pin_FEESER = A2;
//int pot_pin_linser_L = A3;
//int pot_pin_linser_R = A4;
int but_pin_linser_L = 3;
int but_pin_linser_R = 4;


int P16_pos = 0;    // variable to store servo position
int T16_pos = 0;    // variable to store servo position
int FEESER_pos = 0;
int lower_angle = 18;
int upper_angle = 168;
int linser_L_pos = lower_angle;     //linear servo lower responsive angle as 18 deg and upper as 168 deg
int linser_R_pos = upper_angle;

int P16_pin = 9;  // servo pin for FS6530M feetech servo motor
int T16_pin = 10;  // servo pin for T16 actuonix linear actuator
int FEESER_pin = 2;
int linser_L_pin = 0;
int linser_R_pin = 1;


int pot_val_P16;
int pot_val_T16;
int pot_val_FEESER;
//int pot_val_linser_L;
//int pot_val_linser_R;
int but_L_state = 0;
int but_R_state = 0;

int pot_valmap_P16 = 0;
int pot_valmap_T16 = 0;
int pot_valmap_FEESER = 0;
//int pot_valmap_linser_L = 0;
//int pot_valmap_linser_R = 0;


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    pinMode(but_pin_linser_L, INPUT);
    pinMode(but_pin_linser_R, INPUT);
   
    P16.attach(P16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    P16.write(P16_pos);
    T16.attach(T16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    T16.write(T16_pos);
    FEESER.attach(FEESER_pin, 544, 2400);  // attached pin number on Teensy as defined above
    FEESER.write(FEESER_pos);
    linser_L.attach(linser_L_pin, 544, 2400);  // attached pin number on Teensy as defined above
    linser_L.write(linser_L_pos);
    linser_R.attach(linser_R_pin, 544, 2400);  // attached pin number on Teensy as defined above
    linser_R.write(linser_R_pos);
    Serial.printf("P16 Angle Read %d\r\n", P16_pos);
    Serial.printf("T16 Angle Read %d\r\n", T16_pos);
    Serial.printf("FEETECH servo Angle Read %d\r\n", FEESER_pos);
    Serial.printf("linear servo Left Angle Read %d\r\n", linser_L_pos);
    Serial.printf("Linear servo Right Angle Read %d\r\n\n", linser_R_pos);
    delay(100);
}

void loop() {

  pot_val_P16 = analogRead(pot_pin_P16);
  pot_val_T16 = analogRead(pot_pin_T16);
  pot_val_FEESER = analogRead(pot_pin_FEESER);
  //pot_val_linser_L = analogRead(pot_pin_linser_L);
  //pot_val_linser_R = analogRead(pot_pin_linser_R);
  but_L_state = digitalRead(but_pin_linser_L);
  but_R_state = digitalRead(but_pin_linser_R);

  P16_pos = P16.read();
  T16_pos = T16.read();
  FEESER_pos = FEESER.read();
  linser_L_pos = linser_L.read();
  linser_R_pos = linser_R.read();
  
  pot_valmap_P16 = map(pot_val_P16, 0, 1023, 0, 180);  //Map analog value from the sensor to the linear actuator
  pot_valmap_T16 = map(pot_val_T16, 0, 1023, 0, 180);
  pot_valmap_FEESER = map(pot_val_FEESER, 0, 1023, 0, 180);
  //pot_valmap_linser_L = map(pot_val_linser_L, 0, 1023, 0, 180);
  //pot_valmap_linser_R = map(pot_val_linser_R, 0, 1023, 0, 180);
  
  Serial.printf("P16 Angle Read: %d\r\n", P16_pos);
  Serial.printf("Potentiometer control P16: %d\r\n\n", pot_valmap_P16);
  Serial.printf("T16 Angle Read: %d\r\n", T16_pos);
  Serial.printf("Potentiometer control T16: %d\r\n\n", pot_valmap_T16);
  Serial.printf("FEETECH servo Angle Read: %d\r\n", FEESER_pos);
  Serial.printf("Potentiometer control Feetech Servo: %d\r\n\n", pot_valmap_FEESER);

  
  P16.write(pot_valmap_P16);
  T16.write(pot_valmap_T16);
  FEESER.write(pot_valmap_FEESER);
  //linser_L.write(pot_valmap_linser_L);
  //linser_R.write(pot_valmap_linser_R);

  
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
