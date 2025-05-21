
#ifndef DWMSPI_H
#define DWMSPI_H

#include "Arduino.h"
#include "SPI.h"

struct Coordinate {
    int32_t x;
    int32_t y;
};

class DwmSpi
{
    public:
        void dwm_pos_get();
        Coordinate filteredPosition;
        Coordinate position;
        void begin();
        void dwm_upd_rate_set();
        const int PIN_CS = 5;
    private:
};


#endif