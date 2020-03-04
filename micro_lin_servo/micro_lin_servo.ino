#include <PWMServo.h>

PWMServo linser;
int lin_servo_pin = 2;
int pos = linser.read();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  linser.attach(lin_servo_pin, 0, 2000);  //pin number, min pulse, max pulse
  //linser.write(pos);
  Serial.printf("Angle Read %d\r\n", linser.read());
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:  
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(200);                       // wait for a 
    
    if(pos > 1){
       for(pos = pos; pos > 1; pos -= 1){
       linser.write(pos);
       delay(30);
       }
    }
    
    for(pos = 1; pos < 180; pos += 1)  // goes from 1 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      linser.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(30);                       // waits 15ms for the servo to reach the position 
      Serial.printf("Angle Read %d\r\n", linser.read());
    } 
    for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 1 degrees 
    {                                
      linser.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(30);                       // waits 15ms for the servo to reach the position 
      Serial.printf("Angle Read %d\r\n", linser.read());
    } 

    
}
