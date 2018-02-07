#include <Servo.h>  // servo library

int pwm_a = 3;  //PWM control for motor outputs 1 and 2 
int pwm_b = 9;  //PWM control for motor outputs 3 and 4 
int dir_a = 2;  //direction control for motor outputs 1 and 2 
int dir_b = 8;  //direction control for motor outputs 3 and 4 

int i = 0;
int styrSignal = 0;
int motorSignal = 0;
int styrPos = 0;
Servo servo1;  // servo control object

void setup()
{
  Serial.begin(9600); //setup serial communication
  servo1.attach(9); // "attaching" the servo1 object to digital pin 9.
  
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  
  analogWrite(pwm_a, 20);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
  analogWrite(pwm_b, 20);
  
  for(int position = 0; position < 180; position += 1)
  {
    servo1.write(position);  // Move to next position
    delay(20);               // Short pause to allow it to move
  }
  
}

void loop()
{
  if(Serial.available()>4 ){
    styrSignal = Serial.read();
    Serial.print("Styrsignal: ");
    Serial.println(styrSignal);
    delay(20);
    motorSignal = Serial.read();
    Serial.print("Motorsignal: ");
    Serial.println(motorSignal);
    digitalWrite(dir_a, LOW);
    servo1.write(180-styrSignal);  // Move to next position
    delay(20);               // Short pause to allow it to move

    analogWrite(pwm_a, motorSignal);
    
  }
  delay(100);
 
}

//  int position;
//  
//  digitalWrite(dir_a, LOW);
//  for (i=0; i<80; i+=5)
//  {
//    analogWrite(pwm_a, i);
//    delay(500);
//  }
// 
//  
//  for(position = 0; position < 180; position += 1)
//  {
//    servo1.write(position);  // Move to next position
//    delay(20);               // Short pause to allow it to move
//  }
//
//  // Tell servo to go to 0 degrees, stepping by one degree
//
//  for(position = 180; position >= 0; position -= 1)
//  {                                
//    servo1.write(position);  // Move to next position
//    delay(20);               // Short pause to allow it to move
//  }
  
