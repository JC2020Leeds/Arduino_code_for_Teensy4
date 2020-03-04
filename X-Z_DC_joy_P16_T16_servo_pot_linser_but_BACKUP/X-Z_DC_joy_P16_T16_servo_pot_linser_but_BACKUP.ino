#include <PWMServo.h>
#include <Encoder.h>
#include <PID_v1

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
int but_pin_linser_L_and_R = 3;
int sw_pin = 4;
int jx_pin = A8;
int jy_pin = A9;
int x_en_pin_a = 5;
int x_en_pin_b = 6;
int z_en_pin_a = 7;
int z_en_pin_b = 8;
int x_cw = 17;
int x_acw = 18;
int z_cw = 19;
int z_acw = 20;
int enX = 11;
int enZ = 12;

Encoder X_motor_en(x_en_pin_a, x_en_pin_b);     //encoder object and the corresponding pins
Encoder Z_motor_en(z_en_pin_a, z_en_pin_b);

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
int but_L_and_R_state = 0;
int sw_val;
int jx_val;
int jy_val;

int pot_valmap_P16 = 0;
int pot_valmap_T16 = 0;
int pot_valmap_FEESER = 0;
//int pot_valmap_linser_L = 0;
//int pot_valmap_linser_R = 0;
int jx_valmap;
int jy_valmap;

long defXpos  = -999;
long defZpos = -999;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    pinMode(but_pin_linser_L_and_R, INPUT);
    pinMode(sw_pin, INPUT);
    digitalWrite(sw_pin, HIGH);
   
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
    pinMode(enX, OUTPUT);
    pinMode(enZ, OUTPUT);
    pinMode(x_cw, OUTPUT);
    pinMode(x_acw, OUTPUT);
    pinMode(z_cw, OUTPUT);
    pinMode(z_acw, OUTPUT);
    
  // Turn off motors - Initial state    
    digitalWrite(x_cw, LOW);
    digitalWrite(x_acw, LOW);
    digitalWrite(z_cw, LOW);
    digitalWrite(z_acw, LOW);
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
  but_L_and_R_state = digitalRead(but_pin_linser_L_and_R);
  jx_val = analogRead(jx_pin);
  jy_val = analogRead(jy_pin);
  sw_val = digitalRead(sw_pin);

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
  jx_valmap = map(jx_val, 0, 1023, 0, 180);
  jy_valmap = map(jy_val, 0, 1023, 0, 180);

  long newXpos, newZpos;
  newXpos = X_motor_en.read();
  newZpos = Z_motor_en.read();
  
  Serial.printf("P16 Angle Read: %d\r\n", P16_pos);
  Serial.printf("Potentiometer control P16: %d\r\n\n", pot_valmap_P16);
  Serial.printf("T16 Angle Read: %d\r\n", T16_pos);
  Serial.printf("Potentiometer control T16: %d\r\n\n", pot_valmap_T16);
  Serial.printf("FEETECH servo Angle Read: %d\r\n", FEESER_pos);
  Serial.printf("Potentiometer control Feetech Servo: %d\r\n\n", pot_valmap_FEESER);
  Serial.printf("Joystick Switch: %d\r\n", sw_val);
  Serial.printf("Joystick X-axis: %d\r\n", jx_valmap);
  // make serial print motor X encoder position
  Serial.printf("X motor Encoder: %d\r\n", newXpos);
  Serial.printf("Joystick Y-axis: %d\r\n", jy_valmap);
  // make serial print motor Z encoder position
  Serial.printf("Z motor Encoder: %d\r\n", newZpos);
  
  P16.write(pot_valmap_P16);
  T16.write(pot_valmap_T16);
  FEESER.write(pot_valmap_FEESER);
  //linser_L.write(pot_valmap_linser_L);
  //linser_R.write(pot_valmap_linser_R);

  
  //To extend Left shaft and Right shaft to shut the mechanism
  if (but_L_and_R_state == HIGH && linser_L_pos <= lower_angle && linser_R_pos >=upper_angle){
    linser_L.write(upper_angle);
    linser_R.write(lower_angle);
    delay(300);    
  }
  else if (but_L_and_R_state == HIGH && linser_L_pos >= upper_angle && linser_R_pos <=lower_angle){
    linser_L.write(lower_angle);
    linser_R.write(upper_angle);
    delay(300);    
  }
  
  // Set motors speed
  // For PWM maximum possible values are 0 to 255
  analogWrite(enX, 130);
  analogWrite(enZ, 130);



  if (jx_valmap > 30 && jx_valmap <150 && jy_valmap > 30 && jy_valmap <150){
    digitalWrite(x_cw, LOW);
    digitalWrite(x_acw, LOW);
    digitalWrite(z_cw, LOW);
    digitalWrite(z_acw, LOW);
  }

  while (jx_valmap >= 150){
    digitalWrite(x_cw, HIGH);
    digitalWrite(x_acw, LOW);
    digitalWrite(z_cw, LOW);
    digitalWrite(z_acw, LOW);
    jx_valmap = reread_X();
  }

  while (jx_valmap <= 30){
    digitalWrite(x_cw, LOW);
    digitalWrite(x_acw, HIGH);
    digitalWrite(z_cw, LOW);
    digitalWrite(z_acw, LOW);
    jx_valmap = reread_X();     //alternatively put those two lines within the function instead of this line
  }

  while (jy_valmap >= 150){
    digitalWrite(x_cw, LOW);
    digitalWrite(x_acw, LOW);
    digitalWrite(z_cw, HIGH);
    digitalWrite(z_acw, LOW);
    jy_valmap = reread_Z();
  }

  while (jy_valmap <= 30){
    digitalWrite(x_cw, LOW);
    digitalWrite(x_acw, LOW);
    digitalWrite(z_cw, LOW);
    digitalWrite(z_acw, HIGH);
    jy_valmap = reread_Z();
  }


  Serial.printf("Linear servo Left Angle Read: %d\r\n", linser_L_pos);
  //Serial.printf("Potentiometer control linear servo L: %d\r\n\n", pot_valmap_linser_L);
  Serial.printf("Linear servo Right Angle Read: %d\r\n", linser_R_pos);
  //Serial.printf("Potentiometer control linear servo R: %d\r\n\n", pot_valmap_linser_R);
  Serial.printf("Button linear servo L&R: %d\r\n\n", but_L_and_R_state);
  delay(200);




}

  ///Functions~///
///Function for re-reading the joystick x and y value within the motor control while loops
  int reread_X(){
    int jx_val;
    int jx_valmap;
    jx_val = analogRead(jx_pin);
    jx_valmap = map(jx_val, 0, 1023, 0, 180);
    return jx_valmap;
  }

  int reread_Z(){
    int jy_val;
    int jy_valmap;
    jy_val = analogRead(jy_pin);
    jy_valmap = map(jy_val, 0, 1023, 0, 180);
    return jy_valmap;
  }
  ///~Functions///

  ///old codes~///
  //  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    P16.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(30);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    P16.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(30);                       // waits 15ms for the servo to reach the position
//  }

  ///~old codes///


  ///Spare~///
//      if (newXpos != defXpos || newZpos != defZpos) {
//
//    Serial.printf("X = %d\r\n", newXpos);
//    Serial.printf("Z = %d\r\n\n", newZpos);
//    defXpos = newXpos;
//    defZpos = newZpos;
//  }
//  // if a character is sent from the serial monitor,
//  // reset both back to zero.
//  if (Serial.available()) {
//    Serial.read();
//    Serial.printf("Reset both knobs to zero");
//    X_motor_en.write(0);
//    Z_motor_en.write(0);
//
//  }
  ///~Spare///
