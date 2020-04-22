#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(5, 6);
Encoder knobRight(7, 8);


// Motor Z connections
//int enZ = 11;
//int in1 = 17;
//int in2 = 18;
// Motor X connections
int enX = 11;
int in3 = 15;
int in4 = 16;

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
  // Set all the motor control pins to outputs
  //pinMode(enZ, OUTPUT);
  pinMode(enX, OUTPUT);
  //pinMode(in1, OUTPUT);
  //pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  directionControl();
  delay(1000);
  //speedControl();
  //delay(1000);
}


// This function lets you control spinning direction of motors
void directionControl() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  
  // Set motors to maximum speed
  // For PWM maximum possible values are 0 to 255
  //analogWrite(enZ, 255);
  analogWrite(enX, 255);

  // Turn on motor A & B
  //digitalWrite(in1, HIGH);
  //digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(5000);
  
  // Now change motor directions
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, HIGH);
  //digitalWrite(in3, HIGH);
  //digitalWrite(in4, LOW);
  //delay(500);
  
  // Turn off motors
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

    if (newLeft != positionLeft || newRight != positionRight) {

    Serial.printf("Left = %d\r\n", newLeft);
    Serial.printf("Right = %d\r\n\n", newRight);
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.printf("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);

    
  }
}

// This function lets you control speed of the motors
void speedControl() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  
  // Turn on motors
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  // Accelerate from zero to maximum speed    0 to 256
  for (int i = 0; i < 255; i++) {
    //analogWrite(enZ, i);
    analogWrite(enX, i);
    delay(20);
  }
  
  // Decelerate from maximum speed to zero    255 to 0
  for (int i = 255; i >= 0; --i) {
    //analogWrite(enZ, i);
    analogWrite(enX, i);
    delay(20);
  }
  
  // Now turn off motors
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

    if (newLeft != positionLeft || newRight != positionRight) {

    Serial.printf("Left = %d\r\n", newLeft);
    Serial.printf("Right = %d\r\n\n", newRight);
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.printf("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);

    
  }
}



//// This function lets you control spinning direction of motors
//void directionControl() {
//  // Set motors to maximum speed
//  // For PWM maximum possible values are 0 to 255
//  analogWrite(enA, 180);
//  analogWrite(enB, 180);
//
//  // Turn on motor A & B
//  digitalWrite(in1, HIGH);
//  digitalWrite(in2, LOW);
//  digitalWrite(in3, HIGH);
//  digitalWrite(in4, LOW);
//  delay(2000);
//  
//  // Now change motor directions
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, HIGH);
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, HIGH);
//  delay(2000);
//  
//  // Turn off motors
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, LOW);
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, LOW);
//}
//
//// This function lets you control speed of the motors
//void speedControl() {
//  // Turn on motors
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, HIGH);
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, HIGH);
//  
//  // Accelerate from zero to maximum speed    0 to 256
//  for (int i = 0; i < 180; i++) {
//    analogWrite(enA, i);
//    analogWrite(enB, i);
//    delay(20);
//  }
//  
//  // Decelerate from maximum speed to zero    255 to 0
//  for (int i = 180; i >= 0; --i) {
//    analogWrite(enA, i);
//    analogWrite(enB, i);
//    delay(20);
//  }
//  
//  // Now turn off motors
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, LOW);
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, LOW);
//}
