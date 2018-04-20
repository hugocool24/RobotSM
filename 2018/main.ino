/*controller defines*/
float xgv;
float ygv;
float alpha;
const float k = 1.1;
float d;
const float Pi = 3.141593;
const float L = 0.2; //car length
const float vmax = 5; //car max velocity
const float deltamax = 30; //max steering angle [deg]
float kappa;
float u[] = {0.0,90.0};
const float amplitude = 0.5;
const float offset = 1.0;

/*servo defines*/
#include <Servo.h>
Servo myservo;
int reqAngle;
int currAngle = 90;
int val;
int pos;
float holder;

/*motor defines*/
int pwm_pin = 3;  //PWM control for motor outputs 1 and 2 
int dir_pin = 12;  //direction control for motor outputs 1 and 2 
int dc;

/*sensor defines*/
const int nrOfFrontSensors = 3; //nr of sensors to use in the controller ADJUST AS FOR HOW MANY SENSORS CURRENTLY CONNECTED
const int sensorPin[] = {2,7,8}; //array of input pins ADJUST FOR CURRENTLY CONNECTED SENSORS
float rawData[nrOfFrontSensors]; //array to store value coming from sensor
const int sensorAngle[] = {0,-45,45}; //MAKE SURE THE CONNECTED PIN INDEX MATCHES SENSOR ANGLES DEFINED HERE. Angles must be Integers
const int midSensorIndex = 0;
float sensorValue[nrOfFrontSensors] = {0,0,0};  //array to store shifted and inverted value for sensor
const float maxSensorVoltage = 3.0; 
const float minSensorVoltage = 0.4;
const float maxSensorDistance = 70.0;
const float minSensorDistance = 10.0;

/*1st LP filter defines*/
float beta;
float y;
float x;
float dFltrd = 0;
float alphaFltrd = 0;


/*moving average filter defines*/
const int numReadings = 30;
float readings[nrOfFrontSensors][numReadings];      // the readings from the analog input
int readIndex[nrOfFrontSensors] = {0,0,0};              // the index of the current reading
float total[nrOfFrontSensors] = {0.0,0.0,0.0};                  // the running total
float avgSensorValue[nrOfFrontSensors] = {0.0,0.0,0.0};                // the average


/*general*/
int counter = 0;

void setup() {
  
  // put your setup code here, to run once:
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object   
  Serial.begin(9600);
  myservo.write(90);
  
  
  pinMode(pwm_pin, OUTPUT);  //Set control pins to be outputs
  pinMode(dir_pin, OUTPUT);
  pinMode(2, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  digitalWrite(dir_pin, HIGH);
  digitalWrite(9, LOW); //unlock brake
  analogWrite(pwm_pin, 0);

  // initialize moving average memory matrix
  for (int i = 0; i < nrOfFrontSensors; i++) {
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[i][thisReading] = 0;
    }
  }
}

void controller(float u[], float d, float alpha, float L) {
  

 // Serial.println(alpha);

  
  //convert to rad for sin(x) and atan(y) and change sign
  
  alpha = (alpha * Pi)/180.0;
  
  /* compute steering control input */
  kappa = 2.0 * sin(alpha) / (d/100.0);
  u[1] = atan(kappa*L);

  //convert back to degrees
  u[1] = (u[1] * 180.0)/Pi;
  
  if (u[1] > deltamax) {
    u[1] = deltamax;
  } else if (u[1] < -deltamax) {
    u[1] = -deltamax;
  }

  
  u[1] = map(u[1],-deltamax,deltamax,15,180-15);

 //Serial.println(u[1]);
  //Serial.println(u[1]);
  /* motor control signal (very simplistic function of steering angle) */
  
  float angle = u[1]*(Pi/180);
  u[0] =  amplitude*(cos(angle-Pi/2) + offset);
 
  /* normalise throttle input */
  /*u[0] = u[0]/vmax;

  /* end of pure pursuit controller */
  
}

int servoControl(float u[], int& currAngle) {

   
    reqAngle = (int) u[1]; //convert req. angle to integer
    
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
  dCycle = map(dCycle,0,255,150,150);
  
  //send duty cycle
  analogWrite(pwm_pin, dCycle);  
}


void getTargetPoint(float& d, float& alpha, float sensorValue[]) {

  //pick target point (if sensor with angle 0 has max distance, go straight else check for which sensor has max distance)
  //here we will maybe need to do something with the side sensors to try and stabilize the car trajectory to the center of the track
  //when we have no obstacles.

   
    

  float delta = 10.0;
  float epsilon = 10.0;
  if ((sensorValue[midSensorIndex] < maxSensorDistance + epsilon) && (sensorValue[midSensorIndex] > maxSensorDistance - epsilon)) {
    d = maxSensorDistance;
    alpha = sensorAngle[midSensorIndex];

  } else { 
    int maxIndex = 0;
    float maxVal = sensorValue[maxIndex];
    for (int i = 0; i < nrOfFrontSensors; i += 1) { 
      if ((maxVal + delta) < sensorValue[i]){
        maxVal = sensorValue[i];
        maxIndex = i;
      }
    }
  d = maxVal;
  alpha = sensorAngle[maxIndex];   
  }
}

//low pass filter:
//LPF: Y(n) = (1-beta)*Y(n-1) + beta*X(n)
float LPF(float x, float yprev, float beta){
  y = (1-beta)*yprev + beta*x;
  return y;
}

void getSensorValues(float avgSensorValue[], int readIndex[], float total[], float readings[nrOfFrontSensors][numReadings], const int numReadings) {


  //read all sensor values
  for (int i = 0; i < nrOfFrontSensors; i += 1) { 
    rawData[i] = analogRead(sensorPin[i])/200.0;
  }
  
  for (int i = 0; i < nrOfFrontSensors; i += 1) { 

    if(rawData[i] > maxSensorVoltage) {
      sensorValue[i] = minSensorDistance;  
    }
       
    else if(rawData[i] < minSensorVoltage) {
      sensorValue[i] = maxSensorDistance;  
    }
    
    else if (rawData[i]>1.5) {
      sensorValue[i] = (rawData[i]-4.2)/(-0.156);
    } else if (rawData[i]>1.0) {
      sensorValue[i] = (rawData[i]-2.1)/(-0.033);
    } else {
      sensorValue[i] = (rawData[i]-1.4)/(-0.011);
    }
  
  }

  //moving average filter
  for (int i = 0; i < nrOfFrontSensors; i += 1) {
    
    total[i] = total[i] - readings[i][readIndex[i]];
    // read from the sensor:
    readings[i][readIndex[i]] = sensorValue[i];
    // add the reading to the total:
    total[i] = total[i] + readings[i][readIndex[i]];

    // calculate the average:
    avgSensorValue[i] = total[i] / numReadings;

    // advance to the next position in the array:
    readIndex[i] = readIndex[i] + 1;
    
    // if we're at the end of the array...
    if (readIndex[i] >= numReadings) {
      // ...wrap around to the beginning:
      readIndex[i] = 0;
    }
  }
}


//use this to test each sensor, check serial monitor for sensor value
void sensorTest() {
  // read the value from the sensor:
  
  float sensorIn1 = analogRead( sensorPin[0] );
  //float value = 1023.0*(1.0/sensorIn);
  float value1;
  sensorIn1 = sensorIn1/200.0;
  if (sensorIn1>1){
    value1 = (3.5-sensorIn1)*13;
  }
  else {
   value1 = (1.3-sensorIn1)/0.01;
  }
  

  float sensorIn2 = analogRead( sensorPin[1] );
  //float value2 = 1023.0*(1.0/sensorIn2);
  float value2;
  sensorIn2 = sensorIn2/200.0;
  if (sensorIn2>1){
    value2 = (3.5-sensorIn2)*13;
  }
  else {
   value2 = (1.3-sensorIn2)/0.01;
  }

  float sensorIn3 = analogRead( sensorPin[2] );
  //float value3 = 1023.0*(1.0/sensorIn3);
  float value3;
  sensorIn3 = sensorIn3/200.0;
  if (sensorIn3>1){
    value3 = (3.5-sensorIn3)*13;
  }
  else {
   value3 = (1.3-sensorIn3)/0.01;
  }
  /*
  Serial.println("center");
  Serial.println(value1);
  Serial.println();
  Serial.println("right");
  Serial.println(value2);
  Serial.println();
  Serial.println("left");
  Serial.println(value3);
  Serial.println();
  */


}



void loop() {

  getSensorValues(avgSensorValue, readIndex, total, readings, numReadings);
  delay(1); //this will give the sensors time to stabilize a new value. also defines numReadings+calctime ms loop
  counter += 1;
  
  if (counter == numReadings) {
    //Serial.println("center");
    //Serial.println(avgSensorValue[0]);
    //Serial.println("right");
    //Serial.println(avgSensorValue[1]);
    //Serial.println("left");
    //Serial.println(avgSensorValue[2]);
    getTargetPoint(d,alpha,avgSensorValue);
    //Serial.println("d");
    //Serial.println(d);
    //Serial.println("alpha");
    //Serial.println(alpha);
    //dFltrd = LPF(d, dFltrd, 0.1);
    //alphaFltrd = LPF(alpha, alphaFltrd, 0.1);
    controller(u,d,alpha,L);
    motorControl(u,2);

    servoControl(u,currAngle);
    counter = 0;
    
    //Serial.println(holder);
  }
  //sensorTest();
}
