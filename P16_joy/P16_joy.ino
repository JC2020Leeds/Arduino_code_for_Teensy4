#include <PWMServo.h>

PWMServo P16;  // create servo object for controlling servo
                // max 8 servos can be created
                
int jx_pin = A1; //pin for joy stick x axis   A1 pin
int jy_pin = A2;  //pin for joy stick y axis    A2 pin
int sw_pin = 5;
int sw_val;
int jx_val;
int jy_val;
int valmap_x;
int valmap_y;
int speed = 1;
int pos = 0;    // variable to store servo position
int P16_pin = 9;  // servo pin for P16 lin actuator


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(sw_pin, INPUT);
    digitalWrite(sw_pin, HIGH);
    P16.attach(P16_pin, 544, 2400);  // attached pin number on Teensy as defined above
    P16.write(pos);
    Serial.printf("Angle Read %d\r\n", P16.read());
    delay(100);
}

void loop() {

  jx_val = analogRead(jx_pin);
  jy_val = analogRead(jy_pin);
  sw_val = digitalRead(sw_pin);

//  if(jy_val > 542 || jy_val < 482){
    //valmap_x = map(jx_val, 0, 1023, speed, -speed);
    //valmap_y = map(jy_val, 0, 1023, speed, -speed);
    
    //valmap_x = map(jx_val, 0, 1023, -512, 512);
    //valmap_y = map(jy_val, 0, 1023, -512, 512);
        
    valmap_x = map(jx_val, 0, 1023, 0, 180);
    valmap_y = map(jy_val, 0, 1023, 0, 180);
    pos = valmap_x;
//  }  
  
  P16.write(pos);


  Serial.printf("Switch: %d\r\n", sw_val);
  Serial.printf("X-axis: %d\r\n", valmap_x);
  Serial.printf("Y-axis: %d\r\n", valmap_y);
  Serial.printf("Angle Read: %d\r\n\n", P16.read());
  Serial.printf("POS value: %d\r\n\n", pos);
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
