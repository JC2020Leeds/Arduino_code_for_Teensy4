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

PWMServo FEESER;


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


long Sang;

//int FEESER_pos = 0;
//int FEESER_pos = 90;
float FEESER_pos = 90.00;

#define FEESER_pin  2

int pot_val_FEESER;

int pot_valmap_P16 = 0;
int pot_valmap_T16 = 0;
int pot_valmap_FEESER = 0;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    //Serial.begin(115200);
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
    
    FEESER.attach(FEESER_pin, 544, 2400);  // attached pin number on Teensy as defined above
    FEESER.write(FEESER_pos);
    
  // Turn off motors - Initial state    

    Serial.printf("FEETECH servo Angle Read %d\r\n", FEESER_pos);

    delay(100);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  //pot_val_FEESER = analogRead(pot_pin_FEESER);


  FEESER_pos = FEESER.read();

  pot_valmap_FEESER = map(pot_val_FEESER, 0, 1023, 0, 180);   //max angle turn for the servo = 


  Sang = map(FEESER_pos, 0, 180, 0, 200);     //actual angle of rotation of servo was ~200 deg from measurements
  //therefore real angle is 1.111111... (repeating) degree per .read()'s degree
    
  Serial.printf("FEETECH servo Angle Read: %d\r\n", FEESER_pos);

  // turn encoder count into angles
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

    servoAngAdj();
  
  ///////////////////////////////////<~~~IMU-related>///////////////////////////////////////// 


}

  ///Functions~///

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
