#include <QTRSensors.h> //QTR Lib

#define rightPWM 2
#define leftPWM 7   
#define rightMotorA 6
#define rightMotorB 5
#define leftMotorA 3
#define leftMotorB 4   //Motor Driver Digital Pins




int maxSpeed = 240;
int baseSpeed = 180;
int minSpeed = -240;


bool ball = false;  //boolean variable to check availability of ball
bool blackBox = false;
bool whiteBox = false;   //Checks whether bot in black box or white box

double kp = 78.0;
double ki = 0.0;  //Scale values
double kd = 68.0;

int setPosition = 3500;  //Sensor Line setPosition 
int error = 0;
int lastError = 0;

int p=0; 
int intg=0;
int d=0;

bool temp1;

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   22    // emitter is controlled by digital pin 2


// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {25, 26, 27, 28, 29, 30, 31, 32},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

unsigned int position;

boolean noLineIsThere()
{
  temp1=true;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i]==0){
      temp1=temp1 && true; 
    }
    else if(sensorValues[i] == 1) {
      temp1=temp1 && false ;
    }
  }
  return temp1;
}

void readSensors(){
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] < 500)
    {
      sensorValues[i] = 1;
    }
    else
    {    
      sensorValues[i] = 0;
    }
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values
}

/*void checkBall(){
  float distance,duration;
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  duration = pulseIn(ECHO,HIGH);
  distance = duration*0.034/2;
  //r_distance = distance_r;

  Serial.print("Distance: ");
  Serial.print(distance);
  //Serial.print("\t");
  Serial.println();
  if(distance < 14.0){
    ball = true;
    Serial.print("Ball found");
    Serial.println();
  }
  else{
    ball = false;
  }  
}
*/
void goForward(){
  digitalWrite(rightMotorA,HIGH);
  digitalWrite(rightMotorB,LOW);
  digitalWrite(leftMotorA,HIGH);
  digitalWrite(leftMotorB,LOW);
  Serial.println("Going Forward");   
}

void goLeft(){
  
  digitalWrite(rightMotorA,HIGH);
  digitalWrite(rightMotorB,LOW);
  digitalWrite(leftMotorA,LOW);
  digitalWrite(leftMotorB,HIGH);
  Serial.println("Going Left");
}

void goRight(){
  
  digitalWrite(rightMotorA,LOW);
  digitalWrite(rightMotorB,HIGH);
  digitalWrite(leftMotorA,HIGH);
  digitalWrite(leftMotorB,LOW);
  Serial.println("Going RIGHT");
}
void goBack(){
  digitalWrite(rightMotorA,LOW);
  digitalWrite(rightMotorB,HIGH);
  digitalWrite(leftMotorA,LOW);
  digitalWrite(leftMotorB,HIGH);
  Serial.println("Going BAck");
}
void Stop(){
  
  digitalWrite(rightMotorA,LOW);
  digitalWrite(rightMotorB,LOW);
  digitalWrite(leftMotorA,LOW);
  digitalWrite(leftMotorB,LOW);
  Serial.println("stop");
}

void allWhite()
{
  bool temp = false;
  for(int i=0; i< NUM_SENSORS; i++){
    if(sensorValues[i] == 0){
      temp += true;
    }
    else{
      temp += false;
    }
  }
  whiteBox = temp;
}

void allBlack()
{
  bool temp = false;
  for( int i=0; i< NUM_SENSORS; i++){
    if(sensorValues[i] == 1){
      temp += true;
    }
    else{
      temp += false;
    }
  }
  blackBox = temp;
}

void setup() {
  // put your setup code here, to run once:
setMotorPins();
delay(500);
  pinMode(35, OUTPUT);
  pinMode(34, OUTPUT);
  digitalWrite(35, LOW);
  digitalWrite(34, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 600; i++)  // make the calibration take about 10 seconds
  {
    goRight();
    analogWrite(leftPWM,maxSpeed);
    analogWrite(rightPWM,maxSpeed);
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(34, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() 
{
  readSensors();
  calculateError();
  //checkBall();
  //allBlack();
  //allWhite();
  //else{
      
      if(error == 0){
        goForward();
        analogWrite(leftPWM,baseSpeed);
        analogWrite(rightPWM,baseSpeed);
      }
      else{
        followLine();
      }
  //}
   //delay(1000);
}
   
  


void setMotorPins()
{
  pinMode(leftPWM,OUTPUT);
  pinMode(rightPWM,OUTPUT);
  pinMode(rightMotorA,OUTPUT);
  pinMode(rightMotorB,OUTPUT);
  pinMode(leftMotorA,OUTPUT);
  pinMode(leftMotorB,OUTPUT);
}

void followLine()
{
  p = error;
   intg = intg + error;
   d = error - lastError;
   lastError = error;
 
    if(intg > 255){
      intg = 255;
    }

    if(intg < -255){
      intg = -255;
    }

  double turn = (kp * p) + (ki * intg) + (kd * d);

  Serial.println();
  Serial.print("Turn:  ");
  Serial.print(turn);
  Serial.println();
  
  int r_Speed = baseSpeed + turn;
  int l_Speed = baseSpeed - turn;

  if(r_Speed > maxSpeed) 
    r_Speed=maxSpeed;
  else if(r_Speed < minSpeed)
    r_Speed=minSpeed;
  if(l_Speed < minSpeed)
    l_Speed=minSpeed;
  else if(l_Speed > maxSpeed)
    l_Speed=maxSpeed;

    if ( l_Speed>0 && r_Speed>0)
      {
        analogWrite(rightPWM,r_Speed);
        analogWrite(leftPWM,l_Speed);
  
        goForward();

        Serial.println("Go Forward");
        Serial.print("Right Speed:  ");
        Serial.print(r_Speed);
        Serial.println();
        Serial.print("Left Speed:  ");
        Serial.print(l_Speed);
        Serial.println();
        
      }
  else if ( l_Speed>0 && r_Speed<0)
      {
        analogWrite(rightPWM,-r_Speed);
        analogWrite(leftPWM,l_Speed);
  
  
        goRight();
        Serial.println("Go Right");
        Serial.print("Right Speed:  ");
        Serial.print(-r_Speed);
        Serial.println();
        Serial.print("Left Speed:  ");
        Serial.print(l_Speed);
        Serial.println();
      }

      else if ( l_Speed<0 && r_Speed>0)
      {
        analogWrite(rightPWM,r_Speed);
        analogWrite(leftPWM,-l_Speed);
  
        goLeft();

        Serial.println("Go Left");
        Serial.print("Right Speed:  ");
        Serial.print(r_Speed);
        Serial.println();
        Serial.print("Left Speed:  ");
        Serial.print(-l_Speed);
        Serial.println();
      }
}
void calculateError()
{
   error = position - setPosition;
   error = error/800;
   Serial.println();
   Serial.print("Error:  ");
   Serial.print(error);
   Serial.println();
}

