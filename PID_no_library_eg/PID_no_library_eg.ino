#define x_en_pin_a  5
#define x_en_pin_b  6
#define x_cw  16
#define x_acw  17
#define enX  11
long Xang;
Encoder X_motor_en(x_en_pin_a, x_en_pin_b);  
long newXpos = X_motor_en.read();
  Xang = newXpos*360/17712;

//PID constants
double kp = 2
double ki = 5
double kd = 1
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, outputt, setPoint;
double cumError, rateError;
 
void setup(){
        setPoint = 0;                          //set point at zero degrees
}    
 
void loop(){
        input = analogRead(A0);                //read from rotary encoder connected to A0
        outputt = computePID(input);
        delay(100);
        analogWrite(3, outputt);                //control the motor based on PID value

  Serial.printf("input: %d\r\n", input);
}
 
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = Setpoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
