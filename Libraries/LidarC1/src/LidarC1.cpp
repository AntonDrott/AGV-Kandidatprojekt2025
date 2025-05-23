#include "Arduino.h"
#include "LidarC1.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LidarStructs.h"

#define lidarBaud 460800

void Lidar::begin(HardwareSerial& serial, int rx, int tx)
{
    lidarSerial = &serial;
    if(!lidarSerial) return;

    lidarSerial->begin(lidarBaud, SERIAL_8N1, rx, tx);
    //Boot sequence for lidar
    sendCommand(0x25);
    delay(3000);
    sendCommand(0x40);
    delay(5000);
    MotorSpeed(256);
    delay(50);
    sendCommand(0x20);
    delay(200);
    //Clear out esp tx
    lidarSerial->flush();

    if(agvAngle == nullptr) //in case of no angle reference given
    {
        agvAngle = &dummyValue;
    }

    xTaskCreatePinnedToCore(
        Lidar::lidarTask,
        "LidarTask",
        1024, //should be enough mem
        this, //send this into task params
        1,
        nullptr,
        1
    );
}

void Lidar::resetMeasurements()
{
    ScanPoint p;
    p.x = 0;
    p.y = 0;

    std::fill(measurements.begin(),measurements.end(),p);
}

//portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
void Lidar::lidarTask(void* parameter) //FreeRTOS requiers void* for whaterver reason
{
    Lidar* lidar = static_cast<Lidar*>(parameter); //32bit -> 4 bytes total
    byte data[5]; //9 bytes total
    uint16_t angle; //11 bytes total
    uint16_t distance; //13 bytes total
    byte quality; //14 bytes total
    ScanPoint tempPoint; //18 bytes total
    int last = -1;
    int turns = 0;
    bool first = true;
    while(1)
    {
        if(lidar->lidarSerial->available() >= 5)
        {
            lidar->lidarSerial->readBytes(data,5);

            if (((0x01 & data[1]) | (0x03 & data[0])) ^ 0x03 == 0x00)
            {   
                //Bitshift wizardry
                quality = data[0] >> 2;
                if(quality < 2) continue; //ignore low quality measurements
                angle = (((data[2] << 8) | data[1]) >> 1) / 64;
                angle += *(lidar->agvAngle);
                if(angle >= 360) angle = 0; //prevent out of range indexing
                if(angle < 0 ) angle += 360; //wrap lidar back

                //if angle is < than last, new rotation probably started.
                if(!first && angle < last)
                {
                    turns++;
                    if(turns > 15)
                    {
                        lidar->resetMeasurements();
                        turns = 0;
                    }
                    
                }
                first = false;
                last = angle;

                distance = ((data[4] << 8) | data[3])/40;

                lidar->polarMeasurements[angle] = distance;

                /*
                    tempPoint.x = short(cos(float(angle*PI/180)) * distance);
                    tempPoint.y = short(sin(float(-angle*PI/180)) * distance);
                    lidar->measurements[angle] = tempPoint; //store x,y distance in array
                */
            }
            else lidar->lidarSerial->read(); //trash byte
        }
        //If System is running without external positioning system, estimate position relative to environment
        //if(lidar->SLAM) lidar->estimatePosition();
    }
}

void Lidar::MotorSpeed(uint16_t rpm)
{
    uint8_t motor_lsb = rpm & 0xFF;
    uint8_t motor_msb = (rpm >> 8) & 0xFF;
    uint8_t checksum = 0xA5 ^ 0xA8 ^ 0x02 ^ motor_lsb ^ motor_msb;
  
    byte packet[] = {0xA5, 0xA8, 0x02 , motor_lsb, motor_msb, checksum};
    lidarSerial->write(packet,6);
}

/*
void Lidar::estimatePosition()
{
    ScanPoint currentAvg;
    ScanPoint prevAvg;

    for (ScanPoint sp : measurements)
    {
        currentAvg.x += sp.x;
        currentAvg.y += sp.y;
    }
    currentAvg.x = currentAvg.x / 360;
    currentAvg.y = currentAvg.y / 360;

    for (ScanPoint sp : prevMeasurements)
    {
        prevAvg.x += sp.x;
        prevAvg.y += sp.y;
    }
    prevAvg.x = prevAvg.x / 360;
    prevAvg.y = prevAvg.y / 360;

    //position logic
    estimatedX = currentAvg.x + prevAvg.x; //+, change of avg is inverted
    estimatedY = currentAvg.y + prevAvg.y;
    //current mesurement becomes prev, ready for next step estimation;
    memcpy(prevMeasurements,measurements, sizeof(ScanPoint) * 360);


}
    */
