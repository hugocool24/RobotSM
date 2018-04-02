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
const int nrOfFrontSensors = 1; //nr of sensors to use in the controller ADJUST AS FOR HOW MANY SENSORS CURRENTLY CONNECTED
int sensorPin[] = {A0}; //array of input pins ADJUST FOR CURRENTLY CONNECTED SENSORS
int rawData[nrOfFrontSensors]; //array to store value coming from sensor
float sensorAngle[] = {45.0,22.5,0.0,22.5,45.0}; //MAKE SURE THE CONNECTED PIN INDEX MATCHES SENSOR ANGLES DEFINED HERE
float sensorValue[nrOfFrontSensors];  //array to store shifted and inverted value for sensor
float offsetValue[nrOfFrontSensors]; //array to store offset value for each sensor

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

  //send duty cycle !!multiplied by 0.3 because its so fast!!
  analogWrite(pwm_pin, dCycle*0.3);  
}


void getTargetPoint(float& dist, float& angle) {
  int maxSensorDistance = 50;
  //read all sensor values
  for (int i = 0; i < nrOfFrontSensors; pos += 1) { 
    rawData[i] = analogRead(sensorPin[i]);
    //cap data at maxSensorDistance cm MIGHT NEED TO TWEAK
    if(rawData[i] > maxSensorDistance) {
      rawData[i] = maxSensorDistance;
    }
  }

  //smooth data, try first without filtering the data
  //smoothedVal1 =  smooth(rawData1, 0.9, smoothedVal1); 

  //invert and shift sensor data
  for (int i = 0; i < nrOfFrontSensors; pos += 1) { 
  sensorValue[i] = 1000*(1.0/rawData[i]) + offsetValue[i];  
  }

  //Serial.print(sensorValue[0]);
  //Serial.println();

  //pick target point (if sensor with angle 0 has max distance, go straight else check for which sensor has max distance)
  if (sensorValue[2] == maxSensorDistance) {
    dist = maxSensorDistance;
    angle = sensorAngle[2];
  } else { 
    int maxIndex = 0;
    int maxVal = sensorValue[maxIndex];
    for (int i = 0; i < nrOfFrontSensors; pos += 1) { 
      if (maxVal<sensorValue[i]){
        maxVal = sensorValue[i];
        maxIndex = i;
      }
    }
  dist = maxVal;
  angle = sensorAngle[maxIndex];   
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
  int sensorIn = analogRead(sensorPin[0]);
  float value = 1000*(1.0/sensorIn);
  Serial.print(value);
  Serial.println();
}

void loop() {
  getTargetPoint(d,alpha);
  controller(u,d,alpha);
  motorControl(&u[0],1);
  currAngle = servoControl(&u[1],currAngle);
  /*sensorTest();*/

}
