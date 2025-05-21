#include <LidarC1.h>
#include <HardwareSerial.h>
#include <AgvCom.h>
#include <BluetoothSerial.h>
#include <array>
#include <DWMSPI.h>
#include <SPI.h>
#include <ExtendedIMU.h>
#include <Wire.h>
#include<algorithm>
#include<cmath>
//#include <LidarMath.h>



#define RAD_TO_DEG 180 / PI
//---------LIDAR related variables---------
#define TX1 17
#define RX1 16
const int res = 90;
HardwareSerial lidarSerial(1);
std::array<ScanPoint,360> scans; //Used to store lidar measurements
Lidar lidar;

////---------DWM1001 variables---------
DwmSpi dwm;

//---------BlueTooth variables---------
AgvCom com;
BluetoothSerial SerialBT;
int btDelay = 5;

//---------Extended IMU variables---------
ExtendedIMU eIMU(Wire, Serial);
int adjustedAngle = 0;

//---------Ding Unit---------
//bool docking = false;
bool dockingComplete = false;
#define dockEnable 4
#define dockResponse 32
#define dockSensitivity 200

//---------Pathing logic / Mode variables----------
int currentIndex = 0; //current pathing point to track to
int margin = 5; //cm margin to target
ScanPoint pathing_queue[20];//Path queue
Coordinate position; //Current position of AGV
Coordinate targetPosition; //Target from pathing queue
enum MODE : byte {PATHING,MAPPING,RETURNING,IDLE,DOCKING};
MODE mode = IDLE;


//----------Blink display----------
struct BlinkVars
{
  int ontime = 0;
  int offtime = 10;
  bool cycle = true;
};

//----------Moving Avg----------
float alpha = 0.3;
Coordinate prevPos;
Coordinate filteredPos;

static BlinkVars bv;

float xMeter = 0;
float yMeter = 0;
int deltaZero = 0; // delta between true 0 and coordinate-grid 0
int adjustToFieldDelay = 50;

void setup() {

  pinMode(27,INPUT);

  pinMode(27,OUTPUT);
  bv.cycle = true;
  bv.offtime = 100;
  bv.ontime = 100;

  prevPos.x = 0;
  prevPos.y = 0;

  xTaskCreatePinnedToCore(
  blinkTask,
  "BlinkTask",
  1000,
  &bv,
  1,
  NULL,
  1
);

  Serial.begin(115200);
  Serial2.begin(115200,SERIAL_8N1,13,14);

  pinMode(dockEnable, OUTPUT);
  pinMode(dockResponse, INPUT);
  Wire.begin();
  delay(2000);
  Serial.println("Setting up IMU...");

  eIMU.calParams.ax_offset = -0.006;
  eIMU.calParams.ay_offset = -0.007;
  eIMU.calParams.az_offset = -1.038;

  eIMU.calParams.gx_offset = 0.089;
  eIMU.calParams.gy_offset = -0.769;
  eIMU.calParams.gz_offset = 0.703;
  
  eIMU.calParams.mx_offset = 31.365;
  eIMU.calParams.my_offset = -1.787;
  eIMU.calParams.mz_offset = 18.885;

  eIMU.calParams.mx_scale = 1;
  eIMU.calParams.my_scale = 1;
  eIMU.calParams.mz_scale = 1;
  eIMU.alpha = 0.3;
  eIMU.cutoff = 0.8;
  //eIMU.cutoff = 1.0;

  delay(200);
  Serial.println("Initializing Lidar system...");
  lidar.attachAngleVariable(adjustedAngle);
  lidar.begin(lidarSerial, RX1, TX1);
  Serial.println("Initializing eIMU");
  eIMU.begin();
  Serial.println("Done.");
  Serial2.flush();
  Serial.println("Initializing DWM...");
  dwm.begin();
  Serial.println("Done.");
  delay(1000);
  Serial.println("Done.");
  Serial.println("Initializing Bluetooth communication...");
  SerialBT.begin("AGV5");
  delay(100);
  com.MAPRES = 5;
  com.begin(SerialBT);
  Serial.println("Done.");
  delay(1000);
  com.begin(SerialBT);

  //com.lastMsg = "PATH(0:100,80:250,200:320,300)";
    bv.cycle = false;

}

bool adaptive = false;

void loop() {

  if(digitalRead(27))
  {
    eIMU.yawEst = 0;
  }
  // if(com.lastMsg != "")
  // {
  //   Serial.println(com.lastMsg);
  // }
  scans = lidar.getPointMeasurements();
  
  dwm.dwm_pos_get();
  position = dwm.position;
  delay(10);
  dwm.dwm_pos_get();
  position.x += dwm.position.x;
  position.y += dwm.position.y;
  delay(10);
  dwm.dwm_pos_get();
  position.x += dwm.position.x;
  position.y += dwm.position.y;

  position.x /= 3;
  position.y /= 3;

  improvePosition();

  if(com.RESETPATH)
  {
    com.ack();
    //Serial.println("Ok, resetting");
    com.RESETPATH = false;
    //com.sendMessage("RESETPATH");
    //com.sendMessage("Path instructions received:");
    //com.sendMessage(com.lastMsg);
    UpdatePath(com.lastMsg);
    delay(5);
    com.sendMessage("LENGTH: " + String(com.pathLength));
    nextTarget();
    Serial2.println("AUTO_ANGLE_ON");
  }
  // if (com.runTask == true)
  // {
  //   //Serial2.println("RUN");
  //   //Serial2.println("AUTO_ANGLE_ON");
  //   safeSend("RUN");
  //   Serial.println("RUN Received");
  //   com.sendMessage("RUN received.");
  //   safeSend("AUTO_ON");
  //   com.runTask = false; //run handled, reset
  //   com.ack();
  // }
  // else if(com.STOP)
  // {
  //   safeSend("STOP");
  //   com.STOP = false; //stop handled, reset
  //   com.ack();
  // }

  if(Serial2.available() > 0)
  {
    String dummy = Serial2.readStringUntil('\n');
  }

  if(currentIndex == com.pathLength -1)
  {
    adaptive = true;
    Serial2.println("ADAPTIVE_ON");
    margin = 2;
  }
  else
  {
    adaptive = false;
    Serial2.println("ADAPTIVE_OFF");
    Serial2.println("AUTO_ANGLE_ON");
    margin = 5;
  }


  if(Serial.available() > 0)
  {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if(data == "RUN")
    {
      com.runTask = true;
      Serial2.println("AUTO_ON");
      com.STOP = false;
    }
    else if(data == "ZERO")
    {
      eIMU.yawEst = 0;
    }
    else if(data.startsWith("PATH("))
    {
      com.lastMsg = data;
      com.RESETPATH = true; //This emulates the behavior when a path comes in over bluetooth
      com.runTask = true;
      Serial2.println("AUTO_ON");
      com.STOP = false;
    }
    else if(data.startsWith("POS:"))
    {
      data = data.substring(data.indexOf(":")+1);
      int sep = data.indexOf(",");
      position.x = data.substring(0,sep).toInt();
      position.y = data.substring(sep+1).toInt();
      Serial.println("New position set");
    }
    else if(data == "DOCK")
    {
      //dockingComplete = false;
      //docking = true;
      dock();
    }
    else
    {
      Serial2.println(data);
    }
    
    data = "";
  }
  

  //if(abs(position.x/10 - pathing_queue[currentIndex].x) < margin && abs(position.y/10 - pathing_queue[currentIndex].y) < margin  && !docking && com.pathLength > 0 && running && mode != IDLE) //
  if(abs(position.x/10 - pathing_queue[currentIndex].x) < margin && abs(position.y/10 - pathing_queue[currentIndex].y) < margin && com.pathLength > 0) // removed mode != IDLE && mode != DOCKING
  {
    currentIndex++;


    nextTarget();
  }
 else
  {
    digitalWrite(dockEnable,LOW);
  }


  if(com.mapRequest)
  {
    com.ack();
    com.sendPos(0, position.x/10, position.y/10);
    com.sendMap(scans, com.MAPRES);
    Serial.println("Sending Map");

  }

  updateAngle();
  updateBluetooth();
  delay(20);
  if (com.runTask == true)
  {
    //Serial2.println("RUN");
    //Serial2.println("AUTO_ANGLE_ON");
    safeSend("RUN");
    Serial.println("RUN Received");
    com.sendMessage("RUN received.");
    safeSend("AUTO_ON");
    com.runTask = false; //run handled, reset
    com.ack();
  }
  else if(com.STOP)
  {
    safeSend("STOP");
    com.STOP = false; //stop handled, reset
    com.ack();
  }
  delay(20);

}

void updateAngle()
{
  adjustedAngle = (360 - eIMU.yawEst*180/PI);
  if(adjustedAngle < 0) {adjustedAngle += 360;}
  else if(adjustedAngle >= 360){adjustedAngle-= 360;}

  //adjustedAngle = std::clamp(adjustedAngle,0,359);

  //Serial2.println("POS:"+ String(adjustedAngle) + "," + String((position.x/10)) + "," + String(position.y/10));
  Serial2.println("POS:"+ String(adjustedAngle) + "," + String((filteredPos.x)) + "," + String(filteredPos.y));


}

void nextTarget()
{
  //com.sendMessage("Calling nextTarget()");
  com.sendMessage("Target: " + String(pathing_queue[currentIndex].x) + "," + String(pathing_queue[currentIndex].y));
  int newX = pathing_queue[currentIndex].x;
  int newY = pathing_queue[currentIndex].y;
  String msg = "TARGET:" + String(com.receivedAngle) + "," + String(newX) + "," + String(newY);
  safeSend(msg);
  //Serial.println(msg);
  if(currentIndex >= com.pathLength && com.pathLength > 0)
  {
    //currentIndex = 0;
    com.pathLength = 0;
    Serial2.println("AUTO_OFF");

    switch(mode)
    {
      case PATHING:
      dock();

      break;
      case MAPPING:
      //com.sendMap(scans, com.MAPRES);
      mode = IDLE;
      break;
      case RETURNING:
      com.sendMessage("Returned to starting position.");
      mode = IDLE;
      break;
    }

  }

}

void dock()
{
  mode = DOCKING;
  dockingComplete = false;
  Serial2.println("DOCK");
  delay(100); //give motor system time to flush out data
  digitalWrite(dockEnable, HIGH);
  //eIMU.gz = 0;
  int loopLimit = 400;
  int i = 0;
  Serial.print("Attempting to dock.");
  com.sendMessage("Attempting to dock.");

  while(i < loopLimit && !dockingComplete)
  {
    updateAngle();
    if(analogRead(dockResponse) >= dockSensitivity)
    {
      dockingComplete = true;
    }
    Serial.print(".");
    i++;
    delay(100);
    Serial.println(".");
    if(dockingComplete)
    {
      break;
    }
  }

  safeSend("END_DOCK");
  mode = IDLE;

  if(dockingComplete)
  {
    com.ding();
  }
  else
  {
    com.ding();
    com.sendMessage("Timed out.");
  }
}


int i = 0;
void updateBluetooth()
{

  if(i >= btDelay)
  {
    i = 0;
    //com.sendPos(adjustedAngle, position.x/10, position.y/10);
    com.sendPos(adjustedAngle, filteredPos.x , filteredPos.y);
    //com.sendMap(scans, com.MAPRES);
  }
  else
  {
    i++;
  }
}

void UpdatePath(String s)
{
  currentIndex = 0;
  //Serial.println("UpdatePath paramter:");
  //Serial.println(s);
  ScanPoint temp_point;
  int i = s.indexOf("(");
  s = s.substring(i+1); //Remove "PATH(" from data
  int M = s.substring(0,s.indexOf(":")).toInt();
  //com.sendMessage(String(M));
  if(M <= 360)
  {
      com.receivedAngle = M;
      mode = PATHING;
  }
  else if(M == 500)
  {
    mode = MAPPING;
    com.receivedAngle = 0;
  }
  else if(M == 600)
  {
    mode = RETURNING;
    com.receivedAngle = 0;
  }

  s = s.substring(s.indexOf(":")+1);
  int index = 0;

  while(s.length() > 0 )
  { //Data formated as x1 , y1 : x2 , y2 : ... 
    int i  = s.indexOf(",");
    int k = s.indexOf(":");
    int end = s.indexOf(")");
    temp_point.x = s.substring(0,i).toInt();
    if(k != -1)
    {
      temp_point.y = s.substring(i+1,k).toInt();
      s = s.substring(k+1);
    }
    else
    {
      temp_point.y = s.substring(i+1,end).toInt();
      s = "";
    }

    pathing_queue[index] = temp_point;
    index ++;
  }

  com.pathLength = index;
  com.sendMessage("Pathing data received! Lenght:" + String(com.pathLength));
  com.sendMessage("Mode set to: " + printMode(mode));
  com.lastMsg = "";
}

String printMode(MODE m)
{
  switch(m)
  {
  case PATHING: return "PATHING";
  case DOCKING: return "DOCKING";
  case MAPPING: return "MAPPING";
  case IDLE: return "IDLE";
  }

}

void blinkTask(void *params)
{
  BlinkVars* bv = static_cast<BlinkVars*>(params);

  while(true)
  {
    if(bv->cycle)
    {
      digitalWrite(27,HIGH);
      delay(bv->ontime);
      digitalWrite(27,LOW);
      delay(bv->offtime);
    }
    else
    {
      digitalWrite(27,LOW);
      delay(100);
    }
  }
}

void safeSend(String msg)
{
  while(1)
  {
    Serial2.println(msg);
    if(Serial2.available() > 0)
    {
      String data = Serial2.readStringUntil('\n');
      data.trim();

      if(data == msg)
      {
        break;
      }
      else
      {
        Serial.println(data);
      }
      delay(50);
    }
  }
}


void improvePosition()
{
  int dp = sqrt(pow(position.x/10 - prevPos.x, 2) + pow(position.y/10 - prevPos.y, 2));
  alpha = 1.0 - (dp / 5); //5cm noise in general. //CHANGE 5 -> 8

  if(alpha < 0.05) alpha = 0.1;
  if(alpha > 1.0) alpha = 1.0;

  filteredPos.x = int(((prevPos.x)*(1-alpha)) + ((position.x/10) * alpha));
  filteredPos.y = int(((prevPos.y)*(1-alpha)) + ((position.y/10) * alpha));
  prevPos.x = filteredPos.x;
  prevPos.y = filteredPos.y;

  // Serial.print("RawX:");
  // Serial.print(position.x/10);
  // Serial.print(",");
  // Serial.print("RawY:");
  // Serial.print(position.y/10);
  // Serial.print(",");
  // Serial.print("X:");
  // Serial.print(filteredPos.x);
  // Serial.print(",");
  // Serial.print("Y:");
  // Serial.println(filteredPos.y);
}

