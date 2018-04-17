/*controller defines*/
float xgv;
float ygv;
float alpha;
float k = 1.1;
float d;
const float Pi = 3.141593;
float L = 0.2; //car length
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
int dir_pin = 12;  //direction control for motor outputs 1 and 2 
int dc;

/*sensor defines*/
const int nrOfFrontSensors = 3; //nr of sensors to use in the controller ADJUST AS FOR HOW MANY SENSORS CURRENTLY CONNECTED
int sensorPin[] = {2,3,4}; //array of input pins ADJUST FOR CURRENTLY CONNECTED SENSORS
float rawData[nrOfFrontSensors]; //array to store value coming from sensor
int sensorAngle[] = {0,45,-45}; //MAKE SURE THE CONNECTED PIN INDEX MATCHES SENSOR ANGLES DEFINED HERE. Angles must be Integers
const int midSensorIndex = 0;
float sensorValue[nrOfFrontSensors] = {0,0,0};  //array to store shifted and inverted value for sensor
float offsetValue[nrOfFrontSensors] = {0,0,0}; //array to store offset value for each sensor

/*filter defines*/
float beta;
float y;
float x;



void setup() {
  
  // put your setup code here, to run once:
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object   
  Serial.begin(9600);
  myservo.write(90);
  
  
  pinMode(pwm_pin, OUTPUT);  //Set control pins to be outputs
  pinMode(dir_pin, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  digitalWrite(dir_pin, HIGH);
  digitalWrite(9, LOW); //unlock brake
  analogWrite(pwm_pin, 0);

}

void controller(float u[], float d, float alpha, float L) {
  

  Serial.println(alpha);

  
  //convert to rad for sin(x) and atan(y) and change sign
  
  alpha = (alpha * Pi)/180.0;
  
  /* compute steering control input */
  kappa = 2.0 * sin(alpha) / (d/100.0);
  u[1] = -atan(kappa*L);

  //convert back to degrees
  u[1] = (u[1] * 180.0)/Pi;
  
  if (u[1] > deltamax) {
    u[1] = deltamax;
  }

  if (u[1] < -deltamax) {
    u[1] = -deltamax;
  }

  
  u[1] = map(u[1],-deltamax,deltamax,180-35,35);

  Serial.println(u[1]);
  //Serial.println(u[1]);
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
  dCycle = map(dCycle,0,255,120,120);
  
  //send duty cycle
  analogWrite(pwm_pin, dCycle);  
}


void getTargetPoint(float& d, float& alpha) {
  int maxSensorDistance = 80;
  int minSensorDistance = 10;
  
  //read all sensor values
  for (int i = 0; i < nrOfFrontSensors; i += 1) { 
    rawData[i] = analogRead(sensorPin[i])/200.0;
   }
 
    //filter data
  for (int i = 0; i < nrOfFrontSensors; i += 1) {
      sensorValue[i] = LPF(rawData[i], sensorValue[i], 0.5);
    }
  
  for (int i = 0; i < nrOfFrontSensors; i += 1) { 
    if (rawData[i]>1) {
      sensorValue[i] = (3.5-rawData[i])*13;
    } else {
      sensorValue[i] = (1.3-rawData[i])/0.01;
    }
    
    //if distance is over maxSensorDistance we cannot trust the sensor reading so set it to minimum hoping other sensor will be chosen
    if(sensorValue[i] > maxSensorDistance) {
      sensorValue[i] = minSensorDistance;  
    }
       
    if(sensorValue[i] < minSensorDistance) {
      sensorValue[i] = minSensorDistance;  
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

//1st order low pass filter:
//LPF: Y(n) = (1-beta)*Y(n-1) + beta*X(n)
int LPF(float x, float yprev, float beta){
  y = (1-beta)*yprev + beta*x;
  return y;
}

//use this to test each sensor, check serial monitor for sensor value
void sensorTest() {
  // read the value from the sensor:
  Serial.println("center");
  float sensorIn1 = analogRead(A2);
  //float value = 1023.0*(1.0/sensorIn);

  float value1;
  sensorIn1 = sensorIn1/200.0;
  if (sensorIn1>1){
    value1 = (3.5-sensorIn1)*13;
  }
  else {
   value1 = (1.3-sensorIn1)/0.01;
  }
  
  Serial.print(value1);
  Serial.println();

  Serial.println("right");
  float sensorIn2 = analogRead(A3);
  //float value2 = 1023.0*(1.0/sensorIn2);
  float value2;
  sensorIn2 = sensorIn2/200.0;
  if (sensorIn2>1){
    value2 = (3.5-sensorIn2)*13;
  }
  else {
   value2 = (1.3-sensorIn2)/0.01;
  }
  Serial.print(value2);
  Serial.println();

  Serial.println("left");
  float sensorIn3 = analogRead(A4);
  //float value3 = 1023.0*(1.0/sensorIn3);

  float value3;
  sensorIn3 = sensorIn3/200.0;
  if (sensorIn3>1){
    value3 = (3.5-sensorIn3)*13;
  }
  else {
   value3 = (1.3-sensorIn3)/0.01;
  }
  
  Serial.print(value3);
  Serial.println();
}



void loop() {
  getTargetPoint(d,alpha);
  controller(u,d,alpha,L);
  motorControl(u,2);
  currAngle = servoControl(u,currAngle);
  delay(50);
  //sensorTest();
}
