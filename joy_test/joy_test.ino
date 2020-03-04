//#include <Servo.h>

//Servo P16;  // create servo object for controlling servo
                // max 8 servos can be created
                
int jx_pin = A1; //pin for joy stick x axis   A1 pin
int jy_pin = A2;  //pin for joy stick y axis    A2 pin
int sw_pin = 5;
int sw_val;
int jx_val;
int jy_val;
int valmap_x;
int valmap_y;
//int speed = 1;
//int pos = 0;    // variable to store servo position
//int P16_pin = 9;  // servo pin for P16 lin actuator


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(sw_pin, INPUT);
    pinMode(jx_pin, INPUT);
    pinMode(jy_pin, INPUT);
    //    P16.attach(P16_pin, 544, 2400);  // attached pin number on Teensy as defined above
//    P16.write(pos);
//    Serial.print("Angle Read %d\r\n", P16.read());
    delay(100);
}

void loop() {

  sw_val = digitalRead(sw_pin);
  jx_val = analogRead(jx_pin);
  jy_val = analogRead(jy_pin);
  //valmap_x = map(jx_val, 0, 1023, -512, 512);
  //valmap_y = map(jy_val, 0, 1023, -512, 512);
  //valmap_x = map(jx_val, 0, 1023, 0, 180);    //map values to servo angles
  //valmap_y = map(jy_val, 0, 1023, 0, 180);
  valmap_x = map(jx_val, 0, 1023, 1, -1);   //for digital like control (directional only
  valmap_y = map(jy_val, 0, 1023, 1, -1);

  Serial.print("Switch: ");
  Serial.print(sw_val);
  Serial.print("\n");
  Serial.print("X-axis: ");
  Serial.print(valmap_x);
  Serial.print("\n");
  Serial.print("Y-axis: ");
  Serial.print(valmap_y);
  Serial.print("\n\n");

//  Serial.printf("Angle Read %d\r\n\n", P16.read());
  delay(200);  

}
