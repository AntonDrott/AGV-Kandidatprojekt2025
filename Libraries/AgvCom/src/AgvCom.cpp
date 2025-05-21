#include "Arduino.h"
#include "AgvCom.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



void AgvCom::begin(BluetoothSerial& bt)
{
    btSerial = &bt;

     xTaskCreatePinnedToCore( //#SoundCloud rapper name, lil' task X
        AgvCom::backgroundTask,
        "BTBGTask",
        1024, //should be enough mem
        this, //send this into task params
        1,
        nullptr,
        1
        );
}

    void AgvCom::sendMap(std::array<ScanPoint, 360>& points, int resolution)
    {
        mapRequest = false;
        static char buffer[4096];  // Make sure this is large enough
        int pos = snprintf(buffer, sizeof(buffer), "MAP(");
    
        for (int i = 0; i < 360; i++) {
            //Check if i matchen angle-resolution and if point isn't 0,0.
            if (i % resolution == 0 && abs(points[i].x) + abs(points[i].y) != 0) {
                pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                                "%d,%d%s",
                                points[i].x, points[i].y,
                                (i + resolution < 360) ? ":" : "");
                if (pos >= sizeof(buffer)) break; // prevent overflow
            }
        }
    
        strncat(buffer, ")", sizeof(buffer) - strlen(buffer) - 1);
        btSerial->println(buffer);
    }

void AgvCom::sendStatus(int status)
{
    btSerial->println("STATUS("+String(status) + ")");
}

void AgvCom::sendPos(int angle, int x, int y)
{
    btSerial->println("POS(" + String(angle) + "," + String(x) + "," + String(y) + ")");
}

void AgvCom::sendMessage(String msg)
{
    btSerial->println(msg);
}

void AgvCom::backgroundTask(void* parameter)
{
    AgvCom* agvcom = static_cast<AgvCom*>(parameter);

    while(1)
    {
        if(agvcom->btSerial->available() > 0)
        {
            
            String packet = agvcom->btSerial->readStringUntil('\n');
            packet.trim();
            

            if(packet.startsWith("PATH("))
            {
                //agvcom->ack();
                agvcom->lastMsg = packet;
                agvcom->RESETPATH = true;
                /*
                int index = 0;
                int i = packet.indexOf('(');
                int angEnd = packet.indexOf(':');
                packet = packet.substring(i+1);
                agvcom->receivedAngle = packet.substring(0,angEnd).toInt();
                packet = packet.substring(angEnd+1);
                int calcX = 0;
                int calcY = 0;

                while(packet.length()>0)
                {
                    int i = packet.indexOf(',');
                    int j = packet.indexOf(':');
                    int end = packet.indexOf(')');
                    if(j != -1)
                    {
                        calcX = packet.substring(0,i).toInt();
                        calcY = packet.substring(i+1,j).toInt();
                        packet = packet.substring(j+1);
                    }
                    else
                    {
                        calcX = packet.substring(0,i).toInt();
                        calcY = packet.substring(i+1,end).toInt();
                        packet = "";
                    }

                    agvcom->pathingQueue[index].x = calcX;
                    agvcom->pathingQueue[index].y = calcY;
                    index++;
                    agvcom->pathLength = index;
                }
                    */
            }
            else if (packet == "STOP")
            {
                //agvcom->ack();
                agvcom->STOP = true;
            }
            else if (packet.startsWith("MAPRES("))
            {
                agvcom->ack();
                int i = packet.indexOf("(");
                int k = packet.indexOf(")");
                packet = packet.substring(i+1);
                agvcom->MAPRES = packet.substring(0,k).toInt(); 

            }
            else if(packet == "RUN")
            {
                //agvcom->ack();
                agvcom->runTask = true;
            }
            else if(packet == "MAP?")
            {
                //agvcom->ack();
                agvcom->mapRequest = true;
            }
            
            packet = "";

        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

}
