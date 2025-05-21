
#include "ExtendedIMU.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void ExtendedIMU::begin()
{


    IMU.setWire(&Wire);
    IMU.beginAccel();
    IMU.beginGyro();
    IMU.beginMag();
    delay(100);

    xTaskCreatePinnedToCore( //#SoundCloud rapper name, lil' task X
        ExtendedIMU::update,
        "UpdateValues",
        4096, //should be enough mem
        this, //send this into task params
        2,
        nullptr,
        1
    );
}

void ExtendedIMU::calibrate(int N)
{
    Serial.println("Calibrating, Do not move sensor.");
    for(int i = 0; i < N; i++)
    {
        IMU.accelUpdate();
        IMU.gyroUpdate();
        calParams.ax_offset += IMU.accelX();
        calParams.ay_offset += IMU.accelY();
        calParams.az_offset += IMU.accelZ();
        calParams.gx_offset += IMU.gyroX();
        calParams.gy_offset += IMU.gyroY();
        calParams.gz_offset += IMU.gyroZ();
        Serial.print("|");
        delay(5);
    }

    calParams.ax_offset /= N;
    calParams.ay_offset /= N;
    calParams.az_offset /= N;
    calParams.gx_offset /= N;
    calParams.gy_offset /= N;
    calParams.gz_offset /= N;

    Serial.println("Stationary calibration complete!");
    delay(1000);

    Serial.println("Calibration magnetometer, rotate sensor in all directions.");
    delay(1500);
    float mx_min, my_min, mz_min, mx_max, my_max, mz_max, mx, my, mz;

    IMU.magUpdate();
    mx_min = mx_max = IMU.magX();
    my_min = my_max = IMU.magY();
    mz_min = mz_min = IMU.magZ();

    for(int i = 0; i < N; i++)
    {
        IMU.magUpdate();
        mx = IMU.magX();
        my = IMU.magY();
        mz = IMU.magZ();
    
        if (mx < mx_min) mx_min = mx;
        if (mx > mx_max) mx_max = mx;
        if (my < my_min) my_min = my;
        if (my > my_max) my_max = my;
        if (mz < mz_min) mz_min = mz;
        if (mz > mz_max) mz_max = mz;
        delay(10);
    }

    calParams.mx_offset = (mx_max + mx_min) / 2.0;
    calParams.my_offset = (my_max + my_min) / 2.0;
    calParams.mz_offset = (mz_max + mz_min) / 2.0;

    float scale_x_raw = (mx_max - mx_min) / 2.0;
    float scale_y_raw = (my_max - my_min) / 2.0;
    float scale_z_raw = (mz_max - mz_min) / 2.0;
    float avg_scale = (scale_x_raw + scale_y_raw + scale_z_raw) / 3.0;
    Serial.println("Magnetometer Calibration Complete!");
}

void ExtendedIMU::calibrationReport()
{
  Serial.println("------------------- IMU Calibration -------------------");
  Serial.print("Accel Offset [ax, ay, az] = [ ");
  Serial.print(calParams.ax_offset, 3);
  Serial.print(", ");
  Serial.print(calParams.ay_offset, 3);
  Serial.print(", ");
  Serial.print(calParams.az_offset, 3);
  Serial.println(" ]");

  Serial.print("Gyro  Offset [gx, gy, gz] = [ ");
  Serial.print(calParams.gx_offset, 3);
  Serial.print(", ");
  Serial.print(calParams.gy_offset, 3);
  Serial.print(", ");
  Serial.print(calParams.gz_offset, 3);
  Serial.println(" ]");

  Serial.print("Mag   Offset [mx, my, mz] = [ ");
  Serial.print(calParams.mx_offset, 3);
  Serial.print(", ");
  Serial.print(calParams.my_offset, 3);
  Serial.print(", ");
  Serial.print(calParams.mz_offset, 3);
  Serial.println(" ]");

  Serial.print("Mag   Scale  [sx, sy, sz] = [ ");
  Serial.print(calParams.mx_scale, 3);
  Serial.print(", ");
  Serial.print(calParams.my_scale, 3);
  Serial.print(", ");
  Serial.print(calParams.mz_scale, 3);
  Serial.println(" ]");
  Serial.println("--------------------------------------------------------");
}

float ExtendedIMU::ComplimentaryFilterYaw(double alpha) {
    float yawMag = atan2(mx, my);
  
    if (yawMag < 0) {
      yawMag += 2 * PI;
    }
  
    yawEst += gz * dt * DEG_TO_RAD;
    float yawComp = alpha * yawEst + (1.0 - alpha) * yawMag;
  
    yawComp = fmod(yawComp, 2 * PI);
    if (yawComp < 0) {
      yawComp += 2 * PI;
    }
  
    return yawComp;
  }

  void ExtendedIMU::kalman_update() {
    // Prediktion
    yawEst += gz * dt * DEG_TO_RAD;
    yawEst = wrap_angle(yawEst);
    kalman.P += kalman.Q;
  
    // Mätuppdatering
    float yaw_error = yawMag - yawEst;
  
    // Hantera wrap-around
    if (yaw_error > PI) yaw_error -= 2 * PI;
    if (yaw_error < -PI) yaw_error += 2 * PI;
  
    float K = kalman.P / (kalman.P + kalman.R);
    yawEst += K * yaw_error;
    yawEst = wrap_angle(yawEst);
    kalman.P *= (1 - K);
  }

    void ExtendedIMU::update(void* paramter)
  {
    ExtendedIMU* eimu = static_cast<ExtendedIMU*>(paramter);
    while(1)
    {
    unsigned long currentTime = micros();
    eimu->dt = (currentTime - eimu->prevTime) / 1e6;
    eimu->prevTime = currentTime;
  
    // Update IMU data
    eimu->IMU.accelUpdate();
    eimu->IMU.gyroUpdate();
    eimu->IMU.magUpdate();
  
    // Calibrate values
    eimu->ax = eimu->IMU.accelX() - eimu->calParams.ax_offset;
    eimu->ay = eimu->IMU.accelY() - eimu->calParams.ay_offset;
    eimu->az = eimu->IMU.accelZ() - eimu->calParams.az_offset;
    eimu->gx = eimu->IMU.gyroX() - eimu->calParams.gx_offset;
    eimu->gy = eimu->IMU.gyroY() - eimu->calParams.gy_offset;
    eimu->gz = eimu->IMU.gyroZ() - eimu->calParams.gz_offset;
    eimu->mx = (eimu->IMU.magX() - eimu->calParams.mx_offset) * eimu->calParams.mx_scale;
    eimu->my = (eimu->IMU.magY() - eimu->calParams.my_offset) * eimu->calParams.my_scale;
    eimu->mz = (eimu->IMU.magZ() - eimu->calParams.mz_offset) * eimu->calParams.mz_scale;

    eimu->filtered_mx = eimu->alpha * eimu->mx + (1 - eimu->alpha) * eimu->filtered_mx;
    eimu->filtered_my = eimu->alpha * eimu->my + (1 - eimu->alpha) * eimu->filtered_my;

    //eimu->yawMag = atan2(eimu->filtered_mx,eimu->filtered_my);
    // eimu->yawMag = -atan2(eimu->filtered_mx,eimu->filtered_my) - eimu->headingError * PI/180;
    //eimu->yawMag = eimu->wrap_angle(eimu->yawMag);

    if (abs(eimu->gz) > eimu->cutoff){
      eimu->yawEst += eimu->gz * eimu->dt * DEG_TO_RAD;
    }
    
    
    //eimu->kalman_update();
    //vTaskDelay(50);
    delay(50);
    }
  }

  void ExtendedIMU::printInfo()
  {
    Serial.print("YawMag estimation (filtered):");
    Serial.println(yawMag * RAD_TO_DEG);
    Serial.print("Yaw estimation (Kalman):");
    Serial.println(yawEst * RAD_TO_DEG);

  }