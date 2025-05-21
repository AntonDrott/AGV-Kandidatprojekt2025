#include "Arduino.h"
#include "SPI.h"
#include "DWMSPI.h"


void DwmSpi::begin()
{

    SPI.begin();
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    delay(1000);
    dwm_upd_rate_set();
    delay(1000);

    byte req[3] = { 0x14, 0x00, 0x00 };
    byte dummy[3];
  
    digitalWrite(PIN_CS, LOW);
    SPI.transferBytes(req, dummy, 3);
    digitalWrite(PIN_CS, HIGH);

}

void DwmSpi::dwm_upd_rate_set() {
    byte rate[6] = { 0x03, 0x04, 0x01, 0x00, 0x01, 0x00 };
    byte rateDummy[6];
  
    digitalWrite(PIN_CS, LOW);
    SPI.transferBytes(rate, rateDummy, 6);
    digitalWrite(PIN_CS, HIGH);
  }

void DwmSpi::dwm_pos_get()
{
    byte req[32];               //request buffer
    req[0] = 0x02;              //Position request
    memset(&req[1], 0x00, 31);  //set remaining bytes to 0x00
    byte packet[32];            //response buffer

    digitalWrite(PIN_CS, LOW);
    SPI.transferBytes(req, packet, 32);
    digitalWrite(PIN_CS, HIGH);
  
    if (packet[0] == 0x40) {
      position.x = (packet[8] << 24) | (packet[7] << 16) | (packet[6] << 8) | packet[5];
      position.y = (packet[12] << 24) | (packet[11] << 16) | (packet[10] << 8) | packet[9];
    }
  
}