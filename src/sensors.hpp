#ifndef H_CANSAT_SENSORS
#define H_CANSAT_SENSORS

#include <cstdint>

#include <TinyGPS++.h>

#include "pins.h"

#define GPS_BAUD_RATE 9600

struct SensedData {
    uint32_t index;

    uint32_t uptime;
    uint16_t micros;

    float    temperature;
    float    pressure;
    uint16_t vibration;

    float    acceleration[3];
    uint8_t  accelerationStatus;
    float    gyroscope[3];
    uint8_t  gyroscopeStatus;
    
    uint32_t gpsTime;
    uint32_t gpsDate;
    double   gpsLatitude;
    double   gpsLongitude;
    double   gpsAltitude;
};

#endif
