#include <MPU6050.h>
#include <Wire.h>
#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include <PWMServo.h>
#include <Encoder.h>
//#include <PID_v1.h>
#include <PIDController.h>
#include <FastPID.h>
#include <AutoPID.h>



PWMServo P16;  // create servo object for controlling servo
PWMServo T16;  // max 8 servos can be created
PWMServo FEESER;
PWMServo linser_L;
PWMServo linser_R;

    ///////////////////////////////////<IMU-related~~~>/////////////////////////////////////////    
/*
MPU6050 IMU;
#define OUTPUT_READABLE_YAWPITCHROLL
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

*/

//////////////////////////<BASIC_READ_WITHOUT_MPU6050.h>////////////////////////////////
const int MPU_addr=0x68;    //0x68 for SCL0 and SDA0 pin on teensy 4 @@0x69 for SCL1 and SDA1
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265;
int maxVal=402;

double MPU_X_ANG;
double MPU_Y_ANG;
double MPU_Z_ANG;

double MPU_X_ANGmap;


    ///////////////////////////////////<~~~IMU-related>/////////////////////////////////////////    

#define pot_pin_P16  A0 //pin for potentiometer    A0 pin
#define pot_pin_T16  A1
///////////////////////////////#define pot_pin_FEESER  A2
//#define pot_pin_linser_L  A3
//#define pot_pin_linser_R  A4
#define but_pin_linser_L_and_R  3
#define sw_pin  4
#define jx_pin  A8
#define jy_pin  A9
#define x_en_pin_a  5
#define x_en_pin_b  6
#define z_en_pin_a  7
#define z_en_pin_b  8
#define x_cw  16
#define x_acw  17
#define z_cw  20
#define z_acw  21
#define enX  11
#define enZ  12

long Xang, Zang;
long Sang;

int P16_pos = 0;    // variable to store servo position
int T16_pos = 0;    // variable to store servo position
//int FEESER_pos = 0;
//int FEESER_pos = 90;
float FEESER_pos = 90.00;

int lower_angle = 18;
int upper_angle = 168;
int linser_L_pos = lower_angle;     //linear servo lower responsive angle as 18 deg and upper as 168 deg
int linser_R_pos = upper_angle;

#define P16_pin  9  // servo pin for FS6530M feetech servo motor
#define T16_pin  10  // servo pin for T16 actuonix linear actuator
#define FEESER_pin  2
#define linser_L_pin  0
#define linser_R_pin  1

Encoder X_motor_en(x_en_pin_a, x_en_pin_b);     //encoder object and the corresponding pins
Encoder Z_motor_en(z_en_pin_a, z_en_pin_b);

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
    //Serial.begin(9600);
    Serial.begin(115200);
    ///////////////////////////////////<IMU-related~~~>/////////////////////////////////////////    
    /*
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    IMU.initialize();
    //pinMode(INTERRUPT_PIN, INPUT);
    */

    /*
    // supply your own gyro offsets here, scaled for min sensitivity
    IMU.setXGyroOffset(220);
    IMU.setYGyroOffset(76);
    IMU.setZGyroOffset(-85);
    IMU.setZAccelOffset(1788); // 1688 factory default for my test chip
    */

    //////////////////////////<BASIC_READ_WITHOUT_MPU6050.h>////////////////////////////////
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    
    ///////////////////////////////////<~~~IMU-related>/////////////////////////////////////////  
    
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

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  pot_val_P16 = analogRead(pot_pin_P16);
  pot_val_T16 = analogRead(pot_pin_T16);
  //pot_val_FEESER = analogRead(pot_pin_FEESER);
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
  pot_valmap_FEESER = map(pot_val_FEESER, 0, 1023, 0, 180);   //max angle turn for the servo = 
  //pot_valmap_linser_L = map(pot_val_linser_L, 0, 1023, 0, 180);
  //pot_valmap_linser_R = map(pot_val_linser_R, 0, 1023, 0, 180);
  jx_valmap = map(jx_val, 0, 1023, 0, 180);
  jy_valmap = map(jy_val, 0, 1023, 0, 180);

  long newXpos, newZpos;
  newXpos = X_motor_en.read();
  newZpos = Z_motor_en.read();
  Xang = newXpos*360/17712;   //angle cal via countable events per rev on output shaft = 17712
  Zang = newZpos*360/17712;
  Sang = map(FEESER_pos, 0, 180, 0, 200);     //actual angle of rotation of servo was ~200 deg from measurements
  //therefore real angle is 1.111111... (repeating) degree per .read()'s degree
    
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
  // turn encoder count into angles
  Serial.printf("X motor angle: %d\r\n",Xang);
  Serial.printf("Z motor angle: %d\r\n",Zang);
  Serial.printf("Servo motor angle: %d\r\n\n",Sang);

  ///////////////////////////////////<IMU-related~~~>/////////////////////////////////////////   
  /*    // display Euler angles in degrees
  IMU.dmpGetQuaternion(&q, fifoBuffer);
  IMU.dmpGetGravity(&gravity, &q);
  IMU.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
  */

  //////////////////////////<BASIC_READ_WITHOUT_MPU6050.h>////////////////////////////////
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);

       MPU_X_ANG= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
       MPU_Y_ANG= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
       MPU_Z_ANG= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

     Serial.print("AngleX= ");
     Serial.println(MPU_X_ANG);

     Serial.print("AngleY= ");
     Serial.println(MPU_Y_ANG);

     Serial.print("AngleZ= ");
     Serial.println(MPU_Z_ANG);
     Serial.println("-----------------------------------------");
     delay(200);

    
    //converting back to level at 180 deg, ACW 90 deg, CW -90 deg
    MPU_X_ANGmap = map(MPU_X_ANG, 0, 360, -180, 180);
    Serial.println(MPU_X_ANGmap);
    Serial.println("......................");

    servoAngAdj();
  
  ///////////////////////////////////<~~~IMU-related>///////////////////////////////////////// 
  
  P16.write(pot_valmap_P16);
  T16.write(pot_valmap_T16);
  //FEESER.write(pot_valmap_FEESER);

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
  analogWrite(enZ, 255);



  if (jx_valmap > 30 && jx_valmap <150 && jy_valmap > 30 && jy_valmap <150){
    digitalWrite(x_cw, LOW);
    digitalWrite(x_acw, LOW);
    digitalWrite(z_cw, LOW);
    digitalWrite(z_acw, LOW);
  }

  while (jx_valmap >= 150){
      // Set motors speed
  // For PWM maximum possible values are 0 to 255
    analogWrite(enX, 130);
    analogWrite(enZ, 0);
    digitalWrite(x_cw, HIGH);
    digitalWrite(x_acw, LOW);
    digitalWrite(z_cw, LOW);
    digitalWrite(z_acw, LOW);
    jx_valmap = reread_X();
  }

  while (jx_valmap <= 30){
      // Set motors speed
  // For PWM maximum possible values are 0 to 255
    analogWrite(enX, 130);
    analogWrite(enZ, 0);
    digitalWrite(x_cw, LOW);
    digitalWrite(x_acw, HIGH);
    digitalWrite(z_cw, LOW);
    digitalWrite(z_acw, LOW);
    jx_valmap = reread_X();     //alternatively put those two lines within the function instead of this line
  }

  while (jy_valmap >= 150){
      // Set motors speed
  // For PWM maximum possible values are 0 to 255
    analogWrite(enX, 130);
    analogWrite(enZ, 0);    
    digitalWrite(x_cw, LOW);
    digitalWrite(x_acw, LOW);
    digitalWrite(z_cw, HIGH);
    digitalWrite(z_acw, LOW);
    jy_valmap = reread_Z();
  }

  while (jy_valmap <= 30){
      // Set motors speed
  // For PWM maximum possible values are 0 to 255
    analogWrite(enX, 130);
    analogWrite(enZ, 0);    
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

// Put following functions into void loop()
  void initialiseArm(){
    //reset arm to horizontal position and fully retracted
    fullRetraction();
      // adjust x axis motor to horizontal
      ///////////////*********** code to adjust to horizontal angle
        digitalWrite(x_cw, HIGH);
        digitalWrite(x_acw, LOW);
        digitalWrite(z_cw, LOW);
        digitalWrite(z_acw, LOW);
  }

  void flightMode(){
    while (Realsense_guided_location_found ==false){
      // pre-gazebo dynamics simulation estimation of arm pointing dowards position for balanced flight
      // adjust x axis motor to approximately 70deg form horizontal
      if (distance_from_ground >= 2){         // 2m off ground (use double precision; define above)
        /////////////*********** code to adjust ot 70deg form horizontal
        digitalWrite(x_cw, LOW);
        digitalWrite(x_acw, HIGH);
        digitalWrite(z_cw, LOW);
        digitalWrite(z_acw, LOW);
      }
      else if (distance_from_ground < 2){
      // adjust x axis motor to horizontal
      ///////////////*********** code to adjust to horizontal angle
        digitalWrite(x_cw, HIGH);
        digitalWrite(x_acw, LOW);
        digitalWrite(z_cw, LOW);
        digitalWrite(z_acw, LOW);
      }
    }
  }

  
  // Functions to achieve T16 and P16 full extension with P16 react to platform distance change
  void fullExtension(){
    
    //% T16 full extend
    T16.write(0);
    while (T16_pos < 180){
      T16_pos++;
      T16.write(T16_pos);
    }
    
    //% adjusting X axis dc motor angle to find appropriate height (100mm (ie. P16 stroke length dependent))
    //from g-robot base to platform
    //USE PID HERE
      
    
    //% when realsense find suitable location on platform
    
    
    //% lower the g-robot via P16 extension 
      P16.write(0);
      for (P16_pos = 0; P16_pos <= 180; P16_pos++){
        P16.write(P16_pos);
        
      }
  }

  void fullRetraction(){
          //% raise the g-robot via P16 retraction 
    for (P16_pos = 0; P16_pos <= 180; P16_pos++){
      P16.write(P16_pos);
      
    }
    
    //% T16 full retract
    while (T16_pos > 0){
      T16_pos--;
      T16.write(T16_pos);
    }    




  }
*/

  
/*    
  void deploy(){
  // including the entire deployment process (starting with arm fully retracted and parallel to ground)
    //% void fullExtension() + below + void fullRetraction()
    
    //% realsense confirm location then release the linear servos
    //% let g-robot drive off and confirm with pi cam (via OpenCV and tags) then retract back to default mode
    //% ie. arm parallel to ground and fully retracted both T16 and P16 with servo 90 deg to armprocess of deploy and retrieve, using all sensors and actuators
  }

  void retrieve(){

  }

  */

  void servoAngAdj(){
    // MPU6050 to be placed on servo motor
    if (MPU_X_ANGmap >= 90){
      FEESER.write(MPU_X_ANGmap-90);      // code to correct the angle of servo and remain level
    }
    else if (MPU_X_ANGmap <= -90){
      FEESER.write(270+MPU_X_ANGmap);
    }
  }
  ///~Functions///

  /*///old codes~///
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    P16.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    P16.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
  }

  ///~old codes///*/


  /*///Spare~///
      if (newXpos != defXpos || newZpos != defZpos) {

    Serial.printf("X = %d\r\n", newXpos);
    Serial.printf("Z = %d\r\n\n", newZpos);
    defXpos = newXpos;
    defZpos = newZpos;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.printf("Reset both knobs to zero");
    X_motor_en.write(0);
    Z_motor_en.write(0);

  }
  ///~Spare///*/
