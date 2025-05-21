#ifndef LidarC1
#define LidarC1

#include "Arduino.h"
#include "HardwareSerial.h"
#include "shared_structs.h"
#include "LidarStructs.h"

class Lidar
{
    public:
        void begin(HardwareSerial& serial, int rx, int tx);
        void MotorSpeed(uint16_t rpm);
        //Attach angle variable to lidar, must be in degrees
        void attachAngleVariable(int& angleVar) {agvAngle = &angleVar;}
        bool SLAM = false;
        bool POLAR = true;
        int estimatedX = 0;
        int estimatedY = 0;
        std::array<ScanData,360> getPolarMeasurements() const
        {
            std::array<ScanData,360> copy;
            std::copy(std::begin(polarMeasurements), std::end(polarMeasurements), copy.begin());
            return copy;
        }
        std::array<ScanPoint,360> getPointMeasurements() const
        {
            std::array<ScanPoint,360> copy;
            std::copy(std::begin(measurements), std::end(measurements), copy.begin());
            return copy;
        }

    private:
        HardwareSerial* lidarSerial = nullptr;
        std::array<ScanPoint,360> measurements;
        //ScanPoint prevMeasurements[360];
        ScanData polarMeasurements[360];
        void resetMeasurements();
        int* agvAngle = nullptr;
        int dummyValue = 0;
        void sendCommand(byte c)
        {
            byte msg[] = {0xA5, c};
            lidarSerial->write(msg, 2);
        }
        void refreshMeasurements();
        static void lidarTask(void* parameter);
        //void estimatePosition();

};

#endif