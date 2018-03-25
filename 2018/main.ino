/*controller defines*/
float xgv;
float ygv;
float alpha;
float k = 1.1;
float d;
const float Pi = 3.141593;
float L = 1; //car length
float vmax = 1; //car max velocity
float deltamax = 1; //max steering angle
float kappa;
float u[] = {0,90};

/*servo defines*/
#include <Servo.h>
Servo myservo;
int reqAngle;
int currAngle = 90;
int val;
int pos;

/*motor defines*/
int pwm_pin = 3;  //PWM control for motor outputs 1 and 2 
int dir_pin = 2;  //direction control for motor outputs 1 and 2 
int dc;

/*sensor defines*/
int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

/*filter defines*/
float smoothedVal1;



void setup() {
  
  // put your setup code here, to run once:
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object   
  Serial.begin(9600);
  myservo.write(90);
  
  
  pinMode(pwm_pin, OUTPUT);  //Set control pins to be outputs
  pinMode(dir_pin, OUTPUT);
  analogWrite(pwm_pin, 0);

}

void controller(float u[], float d, float alpha) {

  
  /* pure pursuit controller%% */

  /* compute steering control input */
  kappa = 2.0 * sin(alpha) / d;
  u[1] = (float) -atan(kappa);
  if (-atan(kappa) > deltamax) {
    u[1] = deltamax;
  }

  /* motor control signal (very simplistic function of steering angle) */
  u[0] = tanh(Pi - k * Pi * fabs(u[1]));

  /* normalise throttle input */
  u[0] = u[0]/vmax;
  /* end of pure pursuit controller */
}

int servoControl(int reqAngle, int currAngle) {
  
    if(reqAngle > currAngle) 
    {
      for (pos = currAngle; pos <= reqAngle; pos += 1) { 
      myservo.write(pos);              
      delay(5);
      }
    } else if(reqAngle < currAngle) {
      for (pos = currAngle; pos >= reqAngle; pos -= 1) { 
      myservo.write(pos);              
      delay(5);
      }
    } else {
      myservo.write(reqAngle);
    }
    
    currAngle = reqAngle;
    return currAngle;
}

void motorControl(int dCycle, int direction)
{
  //set direction
  if (direction == 1) {
    digitalWrite(dir_pin, LOW);
  }
  
  if (direction == 2) {
    digitalWrite(dir_pin, HIGH);
  }

  //send duty cycle !!!!OBS 0.5!!!!!
  analogWrite(pwm_pin, dCycle*0.3);  
}


void getTargetPoint() {
  //read all sensor values
  int rawData1 = analogRead(sensorPin);

  //smooth data
  smoothedVal1 =  smooth(rawData1, 0.9, smoothedVal1); 

  //invert and shift sensor data
  float offset1 = 0;
  float sensorValue1 = 1000*(1.0/smoothedVal1) + offset1;

  Serial.print(sensorValue1);
  Serial.println();

  //pick target point
  /*check all sensor values, pick the value which has largest distance and send that distance and co
   * corresponding angle to control algorithm
   */
}

int smooth(int data, float filterVal, float smoothedVal){


 if (filterVal > 1){      // check to make sure param's are within range
   filterVal = .99;
 }
 else if (filterVal <= 0){
   filterVal = 0;
 }

 smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

 return (int)smoothedVal;
}

void sensorTest() {
  // read the value from the sensor:
  int sensorValue = analogRead(sensorPin);
  float value = 1000*(1.0/sensorValue);
  Serial.print(value);
  Serial.println();
}

void loop() {
  getTargetPoint();
  /*controller(u,0.3,0.1);*/
  /*motorControl(&u[0],1);*/
  /*currAngle = servoControl(&u[1],currAngle);*/
  /*sensorTest();*/

}
