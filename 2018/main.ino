/*controller defines*/
float xgv;
float ygv;
float alpha;
float k = 1.1;
float d;
const float Pi = 3.141593;
float L = 0.3; //car length
float vmax = 5; //car max velocity
float deltamax = 30; //max steering angle [deg]
float kappa;
float u[] = {0.0,90.0};
float amplitude = 0.5;
float offset = 1.0;

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
const int nrOfFrontSensors = 3; //nr of sensors to use in the controller ADJUST AS FOR HOW MANY SENSORS CURRENTLY CONNECTED
int sensorPin[] = {2,3,4}; //array of input pins ADJUST FOR CURRENTLY CONNECTED SENSORS
int rawData[nrOfFrontSensors]; //array to store value coming from sensor
int sensorAngle[] = {-45,0,45}; //MAKE SURE THE CONNECTED PIN INDEX MATCHES SENSOR ANGLES DEFINED HERE. Angles must be Integers
const int midSensorIndex = 1;
int sensorValue[nrOfFrontSensors] = {0,0,0};  //array to store shifted and inverted value for sensor
int offsetValue[nrOfFrontSensors] = {0,0,0}; //array to store offset value for each sensor

/*filter defines*/
float smoothedVal1;



void setup() {
  
  // put your setup code here, to run once:
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object   
  Serial.begin(9600);
  myservo.write(90);
  
  
  pinMode(pwm_pin, OUTPUT);  //Set control pins to be outputs
  pinMode(dir_pin, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  analogWrite(pwm_pin, 0);

}

void controller(float u[], float d, float alpha) {

  //convert to rad for sin(x) and atan(y)
  alpha = (alpha * Pi)/180.0;
  
  /* compute steering control input */
  kappa = 2.0 * sin(alpha) / d;
  u[1] = (float) -atan(kappa);
  
  //convert back to degrees
  u[1] = (u[1] * 180.0)/Pi;

  if (u[1] > deltamax) {
    u[1] = deltamax;
  }

  if (u[1] < -deltamax) {
    u[1] = -deltamax;
  }

  u[1] = map(u[1],-deltamax,deltamax,0,180);
  
  /* motor control signal (very simplistic function of steering angle) */
  
  float angle = u[1]*(Pi/180);
  u[0] =  amplitude*(cos(angle-Pi/2) + offset);
 
  /* normalise throttle input */
  /*u[0] = u[0]/vmax;

  /* end of pure pursuit controller */
 
}

int servoControl(float u[], int currAngle) {
    
    reqAngle = (int) round(u[1]); //convert req. angle to integer after rounding
    if(reqAngle > currAngle) 
    {
      for (pos = currAngle; pos <= reqAngle; pos += 1) { 
      myservo.write(pos);              
      delay(5); //THESE DELAYS CAN PROBABLY BE LOWERED TO GET FASTER STEERING
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

void motorControl(float u[], int direction)
{
  //set direction
  if (direction == 1) {
    digitalWrite(dir_pin, LOW);
  }
  
  if (direction == 2) {
    digitalWrite(dir_pin, HIGH);
  }

  //construct duty cycle, lowest PWM to get the motor rolling is approx. 150. Max duty cycle is 255. Currently restricted to 200
  int dCycle = (int) round(u[0]*255.0);
  dCycle = map(dCycle,0,255,150,200);
  
  //send duty cycle
  analogWrite(pwm_pin, dCycle);  
}


void getTargetPoint(float& d, float& alpha) {
  int maxSensorDistance = 40;
  
  //read all sensor values
  for (int i = 0; i < nrOfFrontSensors; i += 1) { 
    rawData[i] = analogRead(sensorPin[i]);
   }
 
  //smooth data, try first without filtering the data
  //smoothedVal1 =  smooth(rawData1, 0.9, smoothedVal1); 

  //invert and shift sensor data
  for (int i = 0; i < nrOfFrontSensors; i += 1) { 
    sensorValue[i] = 1023.0*(1.0/rawData[i]) + offsetValue[i];
    if(sensorValue[i] > maxSensorDistance) {
      sensorValue[i] = maxSensorDistance;  
    }
   }
  //pick target point (if sensor with angle 0 has max distance, go straight else check for which sensor has max distance)
  //here we will maybe need to do something with the side sensors to try and stabilize the car trajectory to the center of the track
  //when we have no obstacles.

  if (sensorValue[midSensorIndex] == maxSensorDistance) {
    d = maxSensorDistance;
    alpha = sensorAngle[midSensorIndex];

  } else { 
    int maxIndex = 0;
    int maxVal = sensorValue[maxIndex];
    for (int i = 0; i < nrOfFrontSensors; i += 1) { 
      if (maxVal<sensorValue[i]){
        maxVal = sensorValue[i];
        maxIndex = i;
      }
    }
  d = maxVal;
  alpha = sensorAngle[maxIndex];   
  }

}

//smoothing filter
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

//use this to test each sensor, check serial monitor for sensor value
void sensorTest() {
  // read the value from the sensor:
  Serial.println("left");
  int sensorIn = analogRead(A2);
  float value = 1023.0*(1.0/sensorIn);
  Serial.print(sensorIn);
  Serial.println();

  Serial.println("centre");
  int sensorIn2 = analogRead(A3);
  float value2 = 1023.0*(1.0/sensorIn2);
  Serial.print(sensorIn2);
  Serial.println();

  Serial.println("rigth");
  int sensorIn3 = analogRead(A4);
  float value3 = 1023.0*(1.0/sensorIn3);
  Serial.print(sensorIn3);
  Serial.println();


}

void loop() {
  getTargetPoint(d,alpha);
  controller(u,d,alpha);
  //Serial.println(u[0]);
  //motorControl(u,1);
  //currAngle = servoControl(u,currAngle);
  delay(100); //this is to be tweaked
  //sensorTest();
}
