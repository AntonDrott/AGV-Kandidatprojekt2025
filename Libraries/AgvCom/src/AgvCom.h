#ifndef AgvCom_h
#define AgvCom_h

#include "Arduino.h"
#include "BluetoothSerial.h"
#include "LidarStructs.h"



class AgvCom
{
    public:
        ScanPoint pathingQueue[20];
        void sendMessage(String msg);
        void sendMap(std::array<ScanPoint, 360>& points, int resolution);
        void stop();
        void ding(){sendMessage("DING");}
        static void backgroundTask(void* parameter);
        void begin(BluetoothSerial& bt);
        void ack(){btSerial->println("ACK");}
        void sendPos(int angle, int x, int y);
        void sendStatus(int satus);
        String lastMsg = "";

        //state variables
        bool runTask = false;
        bool STOP = false;
        bool RESETPATH = false;
        bool mapRequest = false;
        int MAPRES = 5;
        int receivedAngle = 0;
        int pathLength = 0;
    private:
        BluetoothSerial* btSerial = nullptr;
        ScanPoint tempPoint;
};

#endif