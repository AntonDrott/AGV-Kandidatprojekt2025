#include <AgvMotors.h>
#include <HardwareSerial.h>
#include <QTRSensors.h>
#define deg2rad 180/PI
#define rad2deg PI/180

HardwareSerial interCom(2);
bool targetFlagSent = false;

AgvMotors agv;

//-------Distance sensor vars-------
QTRSensors qtr;
#define TRIG 14
#define ECHO 13

//------Docking vars-------
int step = 600;
bool dockLoop = false;



float measureDistance()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO,HIGH);
  return (duration*0.0343)/2;

}

void setup() {
  Serial.begin(115200);
  interCom.begin(115200,SERIAL_8N1, 16,17);

  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);

  //Motor pin setup
  agv.motorLF.forward = 22;
  agv.motorLF.backward = 23;

  agv.motorRF.forward = 21;
  agv.motorRF.backward = 19;

  agv.motorLB.forward = 5;
  agv.motorLB.backward = 18;

  agv.motorRB.forward = 25;
  agv.motorRB.backward = 4;

  //Motor adjustment setup
  agv.settings.motor_0 = &agv.motorLF;

  agv.motorLF.k = 0.60804;
  agv.motorRF.k = 0.66332;
  agv.motorLB.k = 0.68844;
  agv.motorRB.k = 0.68844;

  agv.motorLF.offset = 7;
  agv.motorRF.offset = 3;
  agv.motorLB.offset = 5;
  agv.motorRB.offset = 0;

  agv.motorLF.m = 30;
  agv.motorRF.m = 32;
  agv.motorLB.m = 31;
  agv.motorRB.m = 34;

    agv.settings.speed = 0.4;
    agv.settings.pwmFreq = 6000;
    agv.settings.pwmRes = 8;
    agv.settings.on = false;
    agv.settings.angleMargin = 2;
    agv.settings.rotationalAdjustDampening = 1.25;

  Serial.println("Starting up motors...");
    agv.begin();
    delay(1000);
    agv.settings.targetRotation = 0;
    agv.settings.autoAdjustAngle = true;
    agv.settings.angleSnapping = false;
    agv.settings.snappingAngle = 45;
    agv.settings.externalControl = false;
    agv.settings.on = true;
    
}

void loop() { 

  if(interCom.available() > 0)
  {
    String packet = interCom.readStringUntil('\n');
    packet.trim();
    //Serial.print("WRAMI: ");
    //Serial.println(packet);

    if(packet.startsWith("POS:"))
    {
      packet = packet.substring(packet.indexOf(":") +1);
      int i = packet.indexOf(",");
      int j = packet.indexOf(",", i+1);
      int angle = packet.substring(0,i).toInt();
      int x = packet.substring(i+1,j).toInt();
      int y = packet.substring(j+1).toInt();

      agv.setState(angle,x,y);

      //Serial.println(String(agv.agvState.angle) + "deg," + String(agv.agvState.x) + "," + String(agv.agvState.y));
    }
    else if(packet.startsWith("TARGET:"))
    {
      echo(packet);
      //Serial.print("Packet: ");
      //Serial.println(packet);
      int i = packet.indexOf(":");
      packet = packet.substring(i+1);
      //Serial.print("->");
      //Serial.println(packet);
      i = packet.indexOf(",");
      agv.settings.targetRotation = packet.substring(0,i).toInt();
      packet = packet.substring(i+1);
      i = packet.indexOf(",");
      agv.settings.targetX = packet.substring(0,i).toInt();
      packet = packet.substring(i+1);
      agv.settings.targetY = packet.toInt();

      //Serial.print("TARGET SET TO: ");
      //Serial.println(String(agv.settings.targetX) + "," + String(agv.settings.targetY));
    }
    else if(packet == "STOP")
    {
      echo(packet);
      agv.settings.on = false;
      //Serial.println("AGV -> OFF");

    }
    else if(packet == "RUN")
    {
      echo(packet);
      agv.settings.on = true;
      //Serial.println("AGV -> ON");
      agv.settings.speed = 0.4;
    }
    else if(packet == "AUTO_ANGLE_ON")
    {
      agv.settings.autoAdjustAngle = true;
    }
    else if(packet == "AUTO_ANGLE_OFF")
    {
      agv.settings.autoAdjustAngle = false;
    }
    else if(packet == "AUTO_ON")
    {
      echo(packet);
      agv.settings.externalControl = true;
      //Serial.println("AUTO ON");
    }
    else if(packet == "AUTO_OFF")
    {
      echo(packet);
      agv.settings.externalControl = false;
      //Serial.println("AUTO OFF");
    }
    else if(packet == "ADAPTIVE_OFF")
    {
      echo(packet);
      agv.settings.adaptiveSpeed = false;
    }
    else if(packet == "ADAPTIVE_ON")
    {
      echo(packet);
      agv.settings.adaptiveSpeed = true;
    }
    else if(packet == "DOCK")
    {

      agv.settings.externalControl = false;
      agv.settings.autoAdjustAngle = false;
      agv.settings.adaptiveSpeed = false;
      dockLoop = true;
    }
    else if(packet == "END_DOCK")
    {
      dockLoop = false;
      agv.settings.autoAdjustAngle = true;
      agv.settings.adaptiveSpeed = true; 
      //Serial.println("END_DOCK Received");
    }
    packet = "";
  }

  while(dockLoop)
  {
    float spd = agv.settings.speed;
    agv.settings.speed = 0.2;
    agv.settings.adaptiveSpeed = false;
    while(true)
    {
      float dist = measureDistance();
      if(dist > 20 || dist < 2) //CHANGE dist < 2
      {
        float dist = measureDistance();
        agv.settings.speed = 0.3; //CHANGE
        agv.nudge(PI/2, 3000); //CHANGE 4000 -> 3000
        delay(200);
        agv.settings.speed = 0.2;
        while(dist >20 || dist < 2)
        {
          agv.nudge(290*PI/180, 300); //CHANGE
          dist = measureDistance();
          delay(100);
          if(dist <= 20 && dist > 0)
          {
            agv.nudge(270*PI/180,600);
            delay(200);
          }
        }
        
      }
      else if(dist > 4)
      {
        agv.nudge(0,200);
      }
      else if(dist <= 2)
      {
        agv.nudge(PI,400);
      }
      else
      {
        break;
      }
      delay(200);
    }

    while(interCom.available() > 0)
    {
      String packet = interCom.readStringUntil('\n');
      packet.trim();
      //Serial.println(packet);
      echo(packet);
      if(packet == "END_DOCK" || packet == "STOP")
      {
        dockLoop = false;
        break;
      }
      delay(2);
    }
    
    if(!dockLoop)
    {
      agv.nudge(PI, 800);
      delay(500);
      break;
    }

    agv.nudge(90*PI/180, step);
    delay(2*step);
    agv.nudge(270*PI/180,step*2);
    delay(4*step);
    agv.nudge(90*PI/180, step);
    delay(2*step);
    delay(200);
    agv.halt();
    agv.settings.adaptiveSpeed = true;
    agv.settings.speed = spd;
  }

  delay(20);

}

void echo(String msg)
{
  if(!msg.startsWith("POS:"))
  {
    interCom.println(msg);
  }
}