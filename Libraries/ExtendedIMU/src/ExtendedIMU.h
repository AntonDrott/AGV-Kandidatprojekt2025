#ifndef EIMU
#define EIMU

#include "Arduino.h"
#include "Wire.h"
#include "MPU9250_asukiaaa.h"

#define NUM_SAMPLES 1000
#define RAD_TO_DEG 180 / PI
#define SDA_PIN 21
#define SCL_PIN 22

struct Kalman
{
    //Yaw estimate error
    float P = 1.0;
    //Process noise
    float Q = 0.0001;
    //measurement noise
    float R = 0.1;
};

struct CalibrationParameters
{
    float ax_offset = 0, ay_offset = 0, az_offset = 0;
    float gx_offset = 0, gy_offset = 0, gz_offset = 0;
    float mx_offset = 0, my_offset = 0, mz_offset = 0;
    float mx_scale = 0, my_scale = 0, mz_scale = 0;
};

class ExtendedIMU
{
    public:
    ExtendedIMU(TwoWire& wire_pointer, Stream& serial_pointer)
    : Wire(wire_pointer), Serial(serial_pointer) {}
    MPU9250_asukiaaa IMU;
    Kalman kalman;
    float alpha = 0.8;        // (mellan 0 och 1)
    float cutoff = 0;

    void begin();
    CalibrationParameters calParams;
    void calibrate(int N);
    void calibrationReport();
 
    void printInfo();
    float yawEst = 0;
    float yawMag = 0;
    float gyroEstimate = 0;
    float gx=0, gy=0, gz=0;

    private:
    float ComplimentaryFilterYaw(double alpha);
    void kalman_update();
    static void update(void* paramter);
    float wrap_angle(float angle) {
        angle = fmod(angle, 2 * PI);
        if (angle < 0) {
          angle += 2 * PI;
        }
        return angle;
      }
    float mx=0, my=0, mz=0;
    float ax=0, ay=0, az=0;


    float filtered_mx = 0.0;
    float filtered_my = 0.0;

    unsigned long prevTime = 0;

    float dt = 0;

    TwoWire& Wire;
    Stream& Serial;

};

#endif