#include <PIDController.h>
volatile long int encoder_pos = 0;
PIDController pos_pid; 
int motor_value = 255;
#define poten A0     // connected to potentiometer
#define encA 5     // x axis motor encoder A
#define encB 6     // x axis motor encoder B
#define xCW 16     // CW direction
#define xACW 17    // ACW direction
int potenmap;


void setup() {

  Serial.begin(9600);
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);    //encoder reading code for X-axis motor
  pinMode(xCW, OUTPUT);   //direction CW
  pinMode(xACW, OUTPUT);  //direction ACW
  attachInterrupt(digitalPinToInterrupt(encA), encoder, RISING);

  pos_pid.begin();    
  pos_pid.tune(20, 0, 200);    
  pos_pid.limit(-255, 255);
}

void loop() {
  //potenmap = map(poten, 0, 1023, 0, 360);
  //pos_pid.setpoint(potenmap);
  pos_pid.setpoint((int)(((float)analogRead(poten)/1023) * 374.0 ));

   motor_value = pos_pid.compute(encoder_pos);    //// Let the PID compute the value, returns the optimal output
if(motor_value > 0){
  MotorCounterClockwise(motor_value);
}else{
  MotorClockwise(abs(motor_value));
}
  Serial.printf("encoder position: %d\r\n", encoder_pos);
  Serial.printf("motor value: %d\r\n", motor_value);
  
  delay(20);
}



void encoder(){

  if(digitalRead(encB) == HIGH){
    encoder_pos++;
  }else{
    encoder_pos--;
  }
}

void MotorClockwise(int power){
  Serial.printf("power: %d\r\n", power);
  if(power > 100){
  analogWrite(xCW, power);
  digitalWrite(xACW, LOW);
  }else{
    digitalWrite(xCW, LOW);
    digitalWrite(xACW, LOW);
  }
}

void MotorCounterClockwise(int power){
  Serial.printf("power: %d\r\n", power);
  if(power > 100){
  analogWrite(xACW, power);
  digitalWrite(xCW, LOW);
  }else{
    digitalWrite(xCW, LOW);
    digitalWrite(xACW, LOW);
  }
}
