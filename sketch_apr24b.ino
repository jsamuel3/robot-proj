#include <SPI.h>
#include <WiFi101.h>
#include <WifiUdp.h>
#include <Math.h>
#include <LSM303.h>
#include <Wire.h>
//#include "arduino_secrets.h"

#define L 90
#define r 25
#define leftIR 12
#define rightIR 6
#define rightMotorA 10
#define rightMotorB 11
#define leftMotorA 5
#define leftMotorB 9
#define gearRatio 0.013333 //(1/75.8)

float cl = 2*PI*(L);
float cr = 2*PI*(r);
float d = cr/(75.8*2);
float phi = 90*(d/cl);
float dx = (L/2)*sin(phi);
float dy = (L/2)-(L/2)*cos(phi);
float x_world, y_world, phi_world = 0;
float tick_phi = 90*(d/cl);
int cntR, cntL = 0;

float thetaLeft = 0, thetaRight = 0, phiDirection = 0;
float errorLeft = 0, errorRight = 0, errorDirection = 0;
float velocityLeft = 0, velocityRight = 0, angularSpeed = 0;
//int tickLeft = 0, tickRight = 0;
float KpSpeed = 80, KpDirection = 5;

bool pickup = false;
int motorLeft, motorRight;
float IMUheading;

LSM303 compass;


void setup() {
  setupWifi();
  openPort();
  motorSetup();
  IRsetup();
  IMUsetup();
//  digitalWrite(5, HIGH);
}

void loop() {
  checkIMU();
  checkUDP();

  setMotor();
  setMotorRight(10);
  setMotorLeft(10);
  Serial.println(IMUheading);
//  Serial.print("xworld = ");
//  Serial.print(x_world);
//  Serial.print("yworld = ");
//  Serial.print(y_world);
//  Serial.println();
//  delay(1000);

}

void doTickLeft(){
  float dxn = 0;
  float dyn = 0;
  cntL++;
  phi_world -= phi;
  dxn = (dx*cos(phi_world))+(dy*sin(phi_world));
  dyn = (dx*sin(phi_world))+(dy*cos(phi_world));
  x_world+=dxn;
  y_world+=dyn;
}
void doTickRight(){
  float dxn = 0;
  float dyn = 0;
  cntR++;
  phi_world += phi;
  dxn = (dx*cos(phi_world))+(dy*sin(phi_world));
  dyn = (dx*sin(phi_world))+(dy*cos(phi_world));
  x_world+=dxn;
  y_world+=dyn;
  
}
void setupWifi(){
  
}
void openPort(){
  
}
void motorSetup(){
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
}
void IRsetup(){
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftIR), doTickLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightIR), doTickRight, CHANGE);
}
void IMUsetup(){
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-3391, -3607, -3835};
  compass.m_max = (LSM303::vector<int16_t>){+2934, +2655, +2857};
}
void checkIMU(){
  compass.read();
  if(compass.a.y > -12000){
    while(true){
      break;
    }
  }
  IMUheading = compass.heading((LSM303::vector<int>){0, 0, 1});
}
void checkUDP(){
  
}
void setMotor(){
  motorRight = 2*velocityRight + (angularSpeed*L)/(2*r);
  motorLeft = 2*velocityLeft + (angularSpeed*L)/(2*r);
  if(motorLeft > 255){
    motorLeft = 255;
  }else if(motorLeft < 0){
    motorLeft = 0;
  }

  if(motorRight > 255){
    motorRight = 255;
  }else if(motorRight < 0){
    motorRight = 0;
  }

  if(pickup){
    motorLeft = 0;
    motorRight = 0;
  }

  analogWrite(rightMotorA, motorRight);
  analogWrite(rightMotorB, 0);
  analogWrite(leftMotorA, motorLeft);
  analogWrite(leftMotorB, 0);
}
void setMotorLeft(float x){
  thetaLeft = float(cntL) * 0.50 * gearRatio;
  errorLeft = x - thetaLeft;
  velocityLeft = errorLeft * KpSpeed;
//  Serial.println(velocityLeft);
//  cntL = 0;
}
void setMotorRight(float x){
  thetaRight = float(cntR) * 0.50 * gearRatio;
  errorRight = x - thetaRight;
  velocityRight = errorRight * KpSpeed;
//  Serial.println(cntR);
//  cntR = 0;
}
void setDirection(float x){

  phiDirection = 0.5 * phi_world + 0.5* IMUheading;
  errorDirection = x - phiDirection;
  angularSpeed = errorDirection * KpDirection;
  
}
