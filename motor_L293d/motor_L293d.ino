//code works when L293d pin 1 and 9 are connected to 5V input

const int x_cw = 16;
const int x_acw = 17;
const int z_cw = 20;
const int z_acw = 21;

void setup() 
{
  pinMode(x_cw , OUTPUT);
  pinMode(x_acw , OUTPUT);
  pinMode(z_cw , OUTPUT);
  pinMode(z_acw , OUTPUT);

}

void loop()
{
//  digitalWrite(x_cw , HIGH);
//  digitalWrite(x_acw , LOW);
//  digitalWrite(z_cw , LOW);
//  digitalWrite(z_acw , LOW);
//  delay(2000);
  digitalWrite(x_cw , LOW);
  digitalWrite(x_acw , LOW);
  digitalWrite(z_cw , HIGH);
  digitalWrite(z_acw , LOW);
  delay(2000);
//  digitalWrite(x_cw , LOW);
//  digitalWrite(x_acw , HIGH);
//  digitalWrite(z_cw , LOW);
//  digitalWrite(z_acw , LOW);
//  delay(2000);
  digitalWrite(x_cw , LOW);
  digitalWrite(x_acw , LOW);
  digitalWrite(z_cw , LOW);
  digitalWrite(z_acw , HIGH);
  delay(2000);
}
