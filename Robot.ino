#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h> 
#include <Math.h>
#include <LSM303.h>
#include <Wire.h>

#include "arduino_secrets.h" 

#define L 90
#define r 25
#define leftIR 6
#define rightIR 12
#define rightMotorA 10
#define rightMotorB 11
#define leftMotorA 5
#define leftMotorB 9
#define gearRatio 0.013333 //(1/75.8)

float cl = 2*PI*(L);
float cr = 2*PI*(r);
float d = cr/(75.8*2);
float phi = d/(L);
float _phi = (d/cl)*(PI);
float dx = (L/2)*sin(_phi);
float dy = (L/2)-(L/2)*cos(_phi);
float x_world=0, y_world=0, phi_world= 0, phi_world_deg = 0;
int cntR, cntL = 0;

float thetaLeft = 0, thetaRight = 0, phiDirection = 0;
float errorLeft = 0, errorRight = 0, errorDirection = 0;
float velocityLeft = 0, velocityRight = 0, angularSpeed = 0;
float KpSpeed = 10, KpDirection = 1.5;
float desHead=0, desVL=0,desVR = 0;

bool pickup = false, turnLeft = false;
bool turn = false;
int motorLeft, motorRight;
float IMUheading;

LSM303 compass;


///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status

typedef struct rtnPacket{
  double x;
  double y;
  double head;
} rtnPacket;

typedef struct cmdPacket{
  double vel;
  double phi;
  int mode;
} packet;

unsigned int localPort = 5005; 
WiFiUDP udp;

char receiveBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

rtnPacket _rtnPacket;
cmdPacket _cmdPacket;
void sendResponse()
{
  _rtnPacket.x = x_world;
  _rtnPacket.y = y_world;
  _rtnPacket.head = phi_world_deg;
//  _rtnPacket.head = IMUheading;
  udp.beginPacket( udp.remoteIP(), udp.remotePort() );

  char trasmitBuffer[255] = { 0 };

  memcpy( trasmitBuffer, &_rtnPacket, sizeof( rtnPacket ) );

  udp.write( trasmitBuffer, sizeof( rtnPacket ) );
  
  udp.endPacket();
}

void readPacket()
{
  int packetSize = udp.parsePacket();
  int len = udp.read(receiveBuffer, 255);
  if (packetSize)
  {
    Serial.println("packet says: ");
     Serial.print(receiveBuffer);
    memcpy( &_cmdPacket, receiveBuffer, sizeof( cmdPacket ) );
    /*
     * mode 0 = return x,y,phi
     * mode 1 = driving commandd
     * mode 2 = cardinal command
     * mode 3 = stop
     */

    switch ( _cmdPacket.mode )
    {
      case 0:
        Serial.println("mode 0");
        
        sendResponse();

        break;
      case 1:
        Serial.println(_cmdPacket.vel);
        Serial.println(_cmdPacket.phi);

        Serial.println("mode 1");
        //parse driving command
        if(_cmdPacket.phi != 0){
          desHead += _cmdPacket.phi;
          if(desHead >= 360){
            desHead = desHead-360;
          }else if(desHead < 0){
            desHead = desHead+360;
          }
          turn = true;
        }else{
          turn = false;
          desVR = _cmdPacket.vel;
          desVL = _cmdPacket.vel;
        }
        break;
      case 2:
        Serial.println("mode 2");
        //parse cardinal command
        desHead = _cmdPacket.phi;
        break;
      case 3:
        Serial.println("mode 3");
        //stop
        break;
    }
  }
}

void setMotorLeft(float x){
  thetaLeft = float(cntL) * 0.50 * gearRatio;
  errorLeft = x - thetaLeft;
  velocityLeft = errorLeft * KpSpeed;
//  Serial.println(velocityLeft);
  if(!turn){
    cntL = 0;
  }

}
void setMotorRight(float x){
  thetaRight = float(cntR) * 0.50 * gearRatio;
  errorRight = x - thetaRight;
  velocityRight = errorRight * KpSpeed;
//  Serial.println(cntR);
  if(!turn){
    cntR = 0;
  }

}

void setMotor(){
  if(_cmdPacket.mode == 2 || turn ==true){
  if(!turnLeft){
    motorRight = (abs(angularSpeed*L))/(2*r);
    motorLeft = 0;
  }else{
    motorRight = 0;
    motorLeft = (abs(angularSpeed)*L)/(2*r);
    }
  }else{
    motorRight = 2*velocityRight;
    motorLeft = 2*velocityLeft;
  }
  if(motorLeft > 100){
    motorLeft = 100;
  }else if(motorLeft < 0){
    motorLeft = 0;
  }

  if(motorRight > 100){
    motorRight = 100;
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

void setDirection(float x){

//  phiDirection = 0.5 * phi_world + 0.5* IMUheading;
  phiDirection = IMUheading;
  errorDirection = x - phiDirection;
  if(x == 0){
    if(phiDirection >= 0 && phiDirection < 30){
      turnLeft = true;
    }else{
      errorDirection = x - (360 - phiDirection);
      turnLeft = false;
    }
  }else{
    if(errorDirection >= 0){
      turnLeft = true;
    }else{
      turnLeft = false;
    }
  }
  angularSpeed = errorDirection * KpDirection;
}

void doTickLeft(){
  float dxn = 0;
  float dyn = 0;
  cntL++;
  phi_world -= _phi;
  phi_world_deg = phi_world * (180/PI);
  if(phi_world < 0){
    phi_world = phi_world + (2*PI);
    phi_world_deg = phi_world * (180/PI);
  }
  dxn = (dx*cos(phi_world))+(dy*sin(phi_world));
  dyn = (dx*sin(phi_world))+(dy*cos(phi_world));
  x_world+=dxn;
  y_world+=dyn;
}
void doTickRight(){
  float dxn = 0;
  float dyn = 0;
  cntR++;
  phi_world += _phi;
  phi_world_deg = phi_world * (180/PI);
  if(phi_world_deg > 360){
    phi_world = phi_world - (2*PI);
    phi_world_deg = phi_world * (180/PI);
  }
  dxn = (dx*cos(phi_world))+(dy*sin(phi_world));
  dyn = (dx*sin(phi_world))+(dy*cos(phi_world));
  x_world+=dxn;
  y_world+=dyn;
  
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
  compass.m_min = (LSM303::vector<int16_t>){-2822, -2534, -3958};
  compass.m_max = (LSM303::vector<int16_t>){+2867, +2298, +2462};
}

void checkIMU(){
  compass.read();
  if(compass.a.y > -12000){
    desVR = 0;
    desVL = 0;
    desHead = IMUheading;
  }
  IMUheading = compass.heading((LSM303::vector<int>){0, 0, 1});
}

void wifiSetup(){
   while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.print("Connected to Wifi");
  IPAddress ip = WiFi.localIP();
  Serial.print("\nIP Address: ");
  Serial.println(ip);
  udp.begin(localPort);
}

void zero(){
  x_world = 0;
  y_world = 0;
  phi_world = 0;
  phi_world_deg = 0;
  digitalWrite(5, LOW);
}

void setup() {
  WiFi.setPins(8,7,4,2);
  Serial.begin(9600);
  motorSetup();
  zero();
  wifiSetup();

  IRsetup();
  IMUsetup();
  checkIMU();
  phi_world_deg = IMUheading;
  phi_world = IMUheading * PI/180;
}

void loop() {
  readPacket();
  checkIMU();
  setDirection(desHead);
  setMotorRight(desVR);
  setMotorLeft(desVL);
  setMotor();

}
