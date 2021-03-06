#include <AFMotor.h>
#include <Servo.h>

AF_DCMotor armRotate(3);



#define leftEN 22
#define leftMotorA 23
#define leftMotorB 24
#define rightMotorA 26
#define rightMotorB 25
#define rightEN 27

Servo actuatorA,actuatorB,claw;

int channel1 = 28;
int channel2 = 29;
int channel3 = 30;
int channel4 = 31;
int channel5 = 32;
int channel6 = 33;


void setup() {
  // put your setup code here, to run once:
actuatorA.attach(10);
armRotate.run(RELEASE);
armRotate.setSpeed(0);
Serial.begin(9600);
//pinMode(xControlA,OUTPUT);
//pinMode(xControlB,OUTPUT);
setMotorPins();
pinMode(channel6,INPUT);
pinMode(channel5,INPUT);
pinMode(channel4,INPUT);
pinMode(channel3,INPUT);
pinMode(channel2,INPUT);
pinMode(channel1,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  
int c1 = pulseIn(channel1,HIGH,250000);
int c2 = pulseIn(channel2,HIGH,250000);
int c3 = pulseIn(channel3,HIGH,250000);
int c4 = pulseIn(channel4,HIGH,250000);
int c5 = pulseIn(channel5,HIGH,250000);
int c6 = pulseIn(channel6,HIGH,250000);

Serial.print(c1);
Serial.print("\t");
Serial.print(c2);
Serial.print("\t");
Serial.print(c3);
Serial.print("\t");
Serial.print(c4);
Serial.print("\t");
Serial.print(c5);
Serial.print("\t");
Serial.print(c6);

Serial.println();
Serial.println();

if((c1 && c2 && c3 && c4 && c5 && c6) == 0){
  Stop();
  armStop();
}
if(c4 > 1750 && c4 < 2000){
  armRight();
  delay(10);
}
else if(c4 > 1000 && c4 <1300){
  armLeft();
  delay(10);
}

if(c2 > 1750 && c2 <2000){
  goForward();
  if(c1 > 1720 && c1 < 2000){
    goRight();
  }
  else if(c1 > 1000 && c1 < 1300){
    goLeft();
  }
}
else if(c2 > 1000 && c2 < 1300){
  goBack();
  if(c1 > 1720 && c1 < 2000){
    goRight();
  }
  else if(c1 > 1000 && c1 < 1300){
    goLeft();
  }
}
else if(c1 > 1720 && c1 < 2000){
  goRight();
}
else if(c1 > 1000 && c1 < 1300){
  goLeft();
}

else{
  Stop();
  armStop();
  //liftStop();
}
//delay(5);
//delay(10);
//Stop();
  //liftStop();
}

void goBack()
{
  digitalWrite(rightEN,HIGH);
  digitalWrite(leftEN,HIGH);
  digitalWrite(rightMotorA,LOW);
  digitalWrite(rightMotorB,HIGH);
  digitalWrite(leftMotorA,LOW);
  digitalWrite(leftMotorB,HIGH);
}

void goRight()
{
  
  digitalWrite(rightEN,HIGH);
  digitalWrite(leftEN,HIGH);
  digitalWrite(rightMotorA,LOW);
  digitalWrite(rightMotorB,HIGH);
  digitalWrite(leftMotorA,HIGH);
  digitalWrite(leftMotorB,LOW);
}

void goLeft()
{
  
  digitalWrite(rightEN,HIGH);
  digitalWrite(leftEN,HIGH);
  digitalWrite(rightMotorA,HIGH);
  digitalWrite(rightMotorB,LOW);
  digitalWrite(leftMotorA,LOW);
  digitalWrite(leftMotorB,HIGH);
}

void goForward()
{
  
  digitalWrite(rightEN,HIGH);
  digitalWrite(leftEN,HIGH);
  digitalWrite(rightMotorA,HIGH);
  digitalWrite(rightMotorB,LOW);
  digitalWrite(leftMotorA,HIGH);
  digitalWrite(leftMotorB,LOW);
}
void Stop()
{
  digitalWrite(rightEN,LOW);
  digitalWrite(leftEN,LOW);
  digitalWrite(rightMotorA,LOW);
  digitalWrite(rightMotorB,LOW);
  digitalWrite(leftMotorA,LOW);
  digitalWrite(leftMotorB,LOW);
}
void setMotorPins()
{
  pinMode(leftEN,OUTPUT);
  pinMode(leftMotorA,OUTPUT);
  pinMode(leftMotorB,OUTPUT);
  pinMode(rightMotorA,OUTPUT);
  pinMode(rightMotorB,OUTPUT);
  pinMode(rightEN,OUTPUT);
}


void armLeft()
{
  armRotate.run(BACKWARD);
  armRotate.setSpeed(240);
}
void armRight()
{
  armRotate.run(FORWARD);
  armRotate.setSpeed(240);
}
void armStop()
{
  armRotate.run(RELEASE);
  armRotate.setSpeed(0);  
}
/*
void armUp()
{
  actuatorA.write(0);
}
void armDown()
{
  yControl.setSpeed(150);
  yControl.run(BACKWARD);
}
void armMid()
{
  
}
void grabClaw()
{
  claw.setSpeed(200);
  claw.run(FORWARD);
}
void releaseClaw()
{
  claw.setSpeed(200);
  claw.run(BACKWARD);
}
void rotateClawRight()
{
  clawRotate.setSpeed(150);
  clawRotate.run(FORWARD);
}
void rotateClawLeft()
{
  clawRotate.setSpeed(150);
  clawRotate.run(BACKWARD);
}
*/
